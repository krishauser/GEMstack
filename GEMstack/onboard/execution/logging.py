from __future__ import annotations
from ..component import Component
from ...utils import serialization,logging,config,settings
from typing import List,Optional,Dict,Set,Any
import time
import datetime
import os
import subprocess
import numpy as np
import cv2
import requests
from msal import PublicClientApplication
import json
import rosbag
import rospy

class LoggingManager:
    """A top level manager of the logging process.  This is responsible for
    creating log folders, log metadata files, and for replaying components from log
    files."""
    def __init__(self):
        self.log_folder = None        # type: Optional[str]
        self.replayed_components = dict()  # type Dict[str,str]
        self.replayed_topics = dict()  # type Dict[str,str]
        self.rosbag_player = None

        self.logged_components = set() # type: Set[str]
        self.logged_topics = set() # type: Set[str]

        self.component_output_loggers = dict() # type: Dict[str,list]
        self.behavior_log = None
        self.rosbag_process = None
        self.run_metadata = dict()    # type: dict
        self.run_metadata['pipelines'] = []
        self.run_metadata['events'] = []
        self.run_metadata['exit_reason'] = 'unknown'
        self.vehicle_time = None
        self.start_vehicle_time = None
        self.debug_messages = {}
        self.onedrive_manager = None

    def logging(self) -> bool:
        return self.log_folder is not None

    def set_log_folder(self, folder : str) -> None:
        self.log_folder = folder

        #save settings.yaml
        config.save_config(os.path.join(folder,'settings.yaml'),settings.settings())

        #save meta.yaml
        self.run_metadata['start_time'] = time.time()
        self.run_metadata['start_time_human_readable'] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        import subprocess
        git_commit_id = subprocess.check_output(['git','rev-parse','HEAD'])
        self.run_metadata['git_commit_id'] = git_commit_id.decode('utf-8').strip()
        git_branch = subprocess.check_output(['git','rev-parse','--abbrev-ref','HEAD'])
        self.run_metadata['git_branch'] = git_branch.decode('utf-8').strip()
        self.dump_log_metadata()
    
    def close(self):
        if self.behavior_log is not None:
            self.behavior_log.close()
            self.behavior_log = None
        for k,(stdout,stderr) in self.component_output_loggers.items():
            if stdout is not None:
                stdout.close()
            if stderr is not None:
                stderr.close()
        self.component_output_loggers = dict()
    
    def replay_components(self, replayed_components : list, replay_folder : str):
        """Declare that the given components should be replayed from a log folder.

        Further make_component calls to this component will be replaced with
        LogReplay objects.
        """
        #sanity check: was this item logged?
        settings = config.load_config_recursive(os.path.join(replay_folder,'settings.yaml'))
        try:
            logged_components = settings['run']['log']['components']
        except KeyError:
            logged_components = []
        for c in replayed_components:
            if c not in logged_components:
                raise ValueError("Replay component",c,"was not logged in",replay_folder,"(see settings.yaml)")
            self.replayed_components[c] = replay_folder

    def replay_topics(self, replayed_topics : list, replay_folder : str):
        """Declare that the given components should be replayed from a log folder.

        Further make_component calls to this component will be replaced with
        BagReplay objects.
        """
        #sanity check: was this item logged?
        settings = config.load_config_recursive(os.path.join(replay_folder,'settings.yaml'))
        try:
            logged_topics = settings['run']['log']['ros_topics']
        except KeyError:
            logged_topics = []
        for c in replayed_topics:
            if c not in logged_topics:
                raise ValueError("Replay topic",c,"was not logged in",replay_folder,"'s vehicle.bag file (see settings.yaml)")
            self.replayed_topics[c] = replay_folder
        if(not self.rosbag_player):
            self.rosbag_player = RosbagPlayer(replay_folder, self.replayed_topics)

        

        
        
    
    def component_replayer(self, vehicle_interface, component_name : str, component : Component) -> Optional[LogReplay]:
        if component_name in self.replayed_components:
            #replace behavior of class with the LogReplay class
            replay_folder = self.replayed_components[component_name]
            outputs = component.state_outputs()
            rate = component.rate()
            assert rate is not None and rate > 0, "Replayed component {} must have a positive rate".format(component_name)
            return LogReplay(vehicle_interface, outputs,
                                os.path.join(replay_folder,'behavior.json'),
                                rate=rate)
        return None

    def dump_log_metadata(self):
        if not self.log_folder:
            return
        from ...utils import config
        config.save_config(os.path.join(self.log_folder,'meta.yaml'),self.run_metadata)

    def load_log_metadata(self):
        if not self.log_folder:
            return
        from ...utils import config
        self.run_metadata = config.load_config_recursive(os.path.join(self.log_folder,'meta.yaml'))

    def component_stdout_file(self,component_name : str) -> str:
        return os.path.join(self.log_folder,component_name+'.stdout.log')

    def component_stderr_file(self,component_name : str) -> str:
        return os.path.join(self.log_folder,component_name+'.stderr.log')

    def log_vehicle_behavior(self,vehicle_interface) -> VehicleBehaviorLogger:
        if not self.log_folder:
            return
        if self.behavior_log is None:
            self.behavior_log = logging.Logfile(os.path.join(self.log_folder,'behavior.json'),delta_format=True,mode='w')
        return VehicleBehaviorLogger(self.behavior_log,vehicle_interface)
    
    def log_state(self,state_attributes : List[str], rate : Optional[float]=None) -> AllStateLogger:
        if not self.log_folder:
            return
        log_fn = os.path.join(self.log_folder,'state.json')
        return AllStateLogger(state_attributes,rate,log_fn)

    def log_components(self,components : List[str]) -> None:
        """Indicate that the state output of these components should be logged"""
        if not self.log_folder:
            return
        if components:
            if self.behavior_log is None:
                self.behavior_log = logging.Logfile(os.path.join(self.log_folder,'behavior.json'),delta_format=True,mode='w')
        self.logged_components = set(components)

    def log_ros_topics(self, topics : List[str], rosbag_options : str = '') -> Optional[str]:
        if topics:
            command = ['rosbag','record','--output-name={}'.format(os.path.join(self.log_folder,'vehicle.bag'))]
            command += rosbag_options.split()
            command += topics
            self.rosbag_process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            return ' '.join(command)
        return None

    def set_vehicle_time(self, vehicle_time : float) -> None:
        self.vehicle_time = vehicle_time
        if self.start_vehicle_time is None:
            self.start_vehicle_time = vehicle_time

    def event(self, event_description : str):
        """Logs an event to the metadata."""
        self.run_metadata['events'].append({'time':time.time(),'vehicle_time':self.vehicle_time,'description':event_description})
        self.dump_log_metadata()

    def debug(self, component : str, item : str, value : Any) -> None:
        """Logs a debug message to the metadata to be saved as CSV."""
        if not self.log_folder:
            return
        if component not in self.debug_messages:
            self.debug_messages[component] = {}
        if isinstance(value,(list,tuple)):
            for i,v in enumerate(value):
                self.debug(component,item+'['+str(i)+']',v)
        elif isinstance(value,dict):
            for k,v in value.items():
                self.debug(component,item+'.'+str(k),v)
        elif isinstance(value,np.ndarray):
            #if really large, save as npz
            folder = os.path.join(self.log_folder,'debug_{}'.format(component))
            if item not in self.debug_messages[component]:
                self.debug_messages[component][item] = []
                os.mkdir(folder)
            filename = os.path.join(folder,item+'_%03d.npz'%len(self.debug_messages[component][item]))
            np.savez(filename,value)
        elif isinstance(value,cv2.Mat):
            #if really large, save as png
            folder = os.path.join(self.log_folder,'debug_{}'.format(component))
            if item not in self.debug_messages[component]:
                self.debug_messages[component][item] = []
                os.mkdir(folder)
            filename = os.path.join(folder,item+'_%03d.png'%len(self.debug_messages[component][item]))
            cv2.imwrite(filename,value)
        else:
            if item not in self.debug_messages[component]:
                self.debug_messages[component][item] = []
            self.debug_messages[component][item].append((time.time(),self.vehicle_time-self.start_vehicle_time,value))
    
    def debug_event(self, component : str, label : str) -> None:
        """Logs a debug event to the metadata."""
        if not self.log_folder:
            return
        if component not in self.debug_messages:
            self.debug_messages[component] = []
        if label not in self.debug_messages[component]:
            self.debug_messages[component][label] = []
        self.debug_messages[component][label].append((time.time(),self.vehicle_time-self.start_vehicle_time,None))
    
    def dump_debug(self):
        if not self.log_folder:
            return
        for k,v in self.debug_messages.items():
            with open(os.path.join(self.log_folder,k+'_debug.csv'),'w') as f:
                columns = []
                isevent = {}
                for col,vals in v.items(): 
                    if ',' in col:
                        col = '"'+col+'"'
                    columns.append(col+' time')
                    columns.append(col+' vehicle time')
                    if not all(x[2] is None for x in vals):
                        isevent[col] = False
                        columns.append(col)
                    else:
                        isevent[col] = True
                f.write(','.join(columns)+'\n')
                nrows = max((len(v[col]) for col in v), default=0)
                for i in range(nrows):
                    row = []
                    for col,vals in v.items():
                        if i < len(vals):
                            row.append(str(vals[i][0]))
                            row.append(str(vals[i][1]))
                            if not isevent[col]:
                                row.append(str(vals[i][2]))
                        else:
                            row.append('')
                            row.append('')
                            if not isevent[col]:
                                row.append('')
                    f.write(','.join(row)+'\n')

    def pipeline_start_event(self, pipeline_name : str) -> None:
        """Logs a pipeline start event to the metadata."""
        self.run_metadata['pipelines'].append({'time':time.time(),'vehicle_time':self.vehicle_time,'name':pipeline_name})
        self.dump_log_metadata()

    def exit_event(self, description, force = False):
        """Exit main loop event.  If a prior reason was given, this does nothing
        unless force = True."""
        if self.run_metadata['exit_reason'] == 'unknown' or force:
            self.run_metadata['exit_reason'] = description
            self.dump_log_metadata()

    def log_component_update(self, component : str, state : Any, outputs : List[str]) -> None:
         """Component update"""
         if component in self.logged_components and len(outputs)!=0:
            self.behavior_log.log(state, outputs, self.vehicle_time)
    
    def log_component_stdout(self, component : str, msg : List[str]) -> None:
        if not self.log_folder:
            return
        if component not in self.component_output_loggers:
            self.component_output_loggers[component] = [None,None]
        if self.component_output_loggers[component][0] is None:
            self.component_output_loggers[component][0] = open(self.component_stdout_file(component),'w')
        timestr = datetime.datetime.fromtimestamp(self.vehicle_time).strftime("%H:%M:%S.%f")[:-3]
        for l in msg:
            self.component_output_loggers[component][0].write(timestr + ': ' + l + '\n')

    def log_component_stderr(self, component : str, msg : List[str]) -> None:
        if not self.log_folder:
            return
        if component not in self.component_output_loggers:
            self.component_output_loggers[component] = [None,None]
        if self.component_output_loggers[component][1] is None:
            self.component_output_loggers[component][1] = open(self.component_stderr_file(component),'w')
        timestr = datetime.datetime.fromtimestamp(self.vehicle_time).strftime("%H:%M:%S.%f")[:-3]
        for l in msg:
            self.component_output_loggers[component][1].write(timestr + ': ' + l + '\n')
    
    def close(self):
           
        
        self.dump_debug()
        self.debug_messages = {}
        if self.rosbag_process is not None:
            out,err = self.rosbag_process.communicate()  # Will block 
            print('-------------------------------------------')
            print("rosbag output:")
            print(out)
            print()
            loginfo = os.stat(os.path.join(self.log_folder,'vehicle.bag'))
            print("Logged to",os.path.join(self.log_folder,'vehicle.bag'))
            print('Log file size in MegaBytes is {}'.format(loginfo.st_size / (1024 * 1024)))
            print('-------------------------------------------')
            self.rosbag_process = None
            
            record_bag = input("Do you want to upload this Rosbag? Y/N (default: Y): ") or "Y"
            if(record_bag not in ["N", "no", "n", "No"]):
                self.onedrive_manager = OneDriveManager()
                self.onedrive_manager.upload_to_onedrive(self.log_folder)
                

    
    def __del__(self):
        self.close()
            
class OneDriveManager():
    def __init__(self):
        self.config_found = False
        try:
            with open("onedrive_config.json", "r") as f:
                config = json.load(f)
                self.CLIENT_ID = config.get("CLIENT_ID")
                self.TENANT_ID = config.get("TENANT_ID")
                self.DRIVE_ID = config.get("DRIVE_ID")
                self.ITEM_ID = config.get("ITEM_ID")
                self.credentials = True

        except Exception as e:
            print("No Onedrive config file found")


    def upload_to_onedrive(self, log_folder):
       

        AUTHORITY = f'https://login.microsoftonline.com/{self.TENANT_ID}'
        SCOPES = ['Files.ReadWrite.All']

        app = PublicClientApplication(self.CLIENT_ID, authority=AUTHORITY)
        accounts = app.get_accounts()

        print("Opening Authentication Window")

        if accounts:
            result = app.acquire_token_silent(SCOPES, account=accounts[0])
        else:

            result = app.acquire_token_interactive(SCOPES)

        if 'access_token' in result:
            access_token = result['access_token']
            headers = {
                'Authorization': f'Bearer {access_token}',
                'Content-Type': 'application/octet-stream'
            }
            file_path = os.path.join(log_folder,  'vehicle.bag')
            file_name = log_folder[5:]+ "_" +  os.path.basename(file_path)
            upload_url = (
            f'https://graph.microsoft.com/v1.0/drives/{self.DRIVE_ID}/items/'
            f'{self.ITEM_ID}:/{file_name}:/content'
            )

            with open(file_path, 'rb') as file:
                response = requests.put(upload_url, headers=headers, data=file)

            if response.status_code == 201 or response.status_code == 200:
                print(f"✅ Successfully uploaded '{file_name}' to OneDrive.")
            else:
                print(f"❌ Upload failed: {response.status_code} - {response.text}")
        else:
            print("❌ Authentication failed.")


class LogReplay(Component):
    """Substitutes the output of a component with replayed data from a log file.

    There are two forms of log files supported.  The first is a delta format, where
    each line is a dictionary of the form ``{'time':t,ITEM1:{...},ITEM2:{...}}``. 
    
    The second is a state format, where each line is a dictionary of the form
    ``{ITEM1:VAL, ITEM2:VAL, ITEM1_update_time:t, ITEM2_update_time:t}``.
    
    If the `delta_format` attribute is True, then the delta format is assumed.
    """
    def __init__(self, vehicle_interface, outputs : List[str],
                 log_file : str,
                 delta_format=True,
                 rate : float = 10.0,
                 speed_multiplier : float = 1.0):
        self.vehicle_interface = vehicle_interface
        self.outputs = outputs
        self.logfn = log_file
        self._rate = rate
        self.speed_multiplier = speed_multiplier
        self.logfile = logging.Logfile(log_file,delta_format,'r')
        self.start_time = None
    
    def rate(self):
        return self._rate
   
    def state_outputs(self):
        return self.outputs

    def update(self):
        t = self.vehicle_interface.time()
        if self.start_time == None:
            self.start_time = t
        if not self.logfile:
            return

        res,msgs = self.logfile.read(duration_from_start = (t - self.start_time)*self.speed_multiplier, cumulative = True)
        #if nothing new was read, just return None
        if len(msgs)==0:
            return None
        #convert the dict to a list of values in the same order as self.outputs
        res = [res.get(o,None) for o in self.outputs]
        if len(self.outputs)==1:
            return res[0]
        if all(v is None for v in res):
            return None
        return res

    def cleanup(self):
        self.logfile.close()


class RosbagPlayer:
    '''
    Class which manages Ros bag replay. Note that this does not work unless the executor is running
    '''
    def __init__(self, bag_path, topics):
        self.bag = rosbag.Bag(os.path.join(bag_path, 'vehicle.bag'), 'r')
        self.current_time = None 
        self.offset = -1

        self.publishers = {}
        for topic, msg, _ in self.bag.read_messages():
            if topic in topics and topic not in self.publishers:
                msg_type = type(msg)
                self.publishers[topic] = rospy.Publisher(topic, msg_type, queue_size=10)
                rospy.loginfo(f"Created publisher for topic: {topic}")
                print(f"Created publisher for topic: {topic}")


    def update_topics(self, target_timestamp):
        """
        Plays from the current position in the bag to the target timestamp.
        :param target_timestamp: gem stack time to play until. 
        Will remember the currenttime and only play from current time to the target timestamp
        """
        if self.offset <0:
            self.offset = target_timestamp - self.bag.get_start_time()
        if self.current_time is None:
            self.current_time = self.bag.get_start_time()

        first_message = True
        for topic, msg, t in self.bag.read_messages(start_time=rospy.Time(self.current_time )):
            if t.to_sec() + self.offset > target_timestamp:
                break  # Stop when reaching the target time
            if first_message and t.to_sec() == self.current_time:
                first_message = False
                continue
            if topic in self.publishers:
                self.publishers[topic].publish(msg)

            self.current_time = t.to_sec()  

    def close(self):
        self.bag.close()

class VehicleBehaviorLogger(Component):
    def __init__(self,behavior_log, vehicle_interface):
        if isinstance(behavior_log,str):
            behavior_log = logging.Logfile(behavior_log,delta_format=True,mode='w')
        self.behavior_log = behavior_log
        self.vehicle_interface = vehicle_interface
        self.vehicle_log_t_last = None

    def rate(self):
        return None

    def state_inputs(self):
        return ['all']
    
    def state_outputs(self):
        return []

    def update(self,state):
        if state.t != self.vehicle_log_t_last:
            collection = {'vehicle_interface_command':self.vehicle_interface.last_command,
                        'vehicle_interface_reading':self.vehicle_interface.last_reading}
            self.behavior_log.log(collection,t=state.t)
            self.vehicle_log_t_last = state.t


class AllStateLogger(Component):
    def __init__(self,attributes,rate,log_fn):   
        self._rate = rate     
        self.attributes = attributes
        self.state_log = logging.Logfile(log_fn,delta_format=False,mode='w')

    def rate(self):
        return self._rate

    def state_inputs(self):
        return ['all']
    
    def state_outputs(self):
        return []
    
    def cleanup(self):
        if self.state_log:
            self.state_log.close()
            self.state_log = None

    def update(self,state):
        if self.attributes:
            if self.attributes[0] == 'all':
                self.state_log.log(state)
            else:
                self.state_log.log(state,self.attributes)
