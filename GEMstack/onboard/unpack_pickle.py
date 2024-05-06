
from enum import Enum
from typing import Tuple,List,Optional
import pickle, json
import os

# ---------------------------------------------------------
# CONFIG
FOLDERPATH = ""

# ---------------------------------------------------------
# ADD SOME BASICS SO WE DON'T HAVE TO IMPORT ALL OF GEMSTACK
class AgentEnum(Enum):
    CAR = 0
    MEDIUM_TRUCK = 1
    LARGE_TRUCK = 2
    PEDESTRIAN = 3
    BICYCLIST = 4

class AgentActivityEnum(Enum):
    STOPPED = 0         # standing pedestrians, parked cars, etc. No need to predict motion.
    MOVING = 1          # standard motion.  Predictions will be used here
    FAST = 2            # indicates faster than usual motion, e.g., runners.
    UNDETERMINED = 3    # unknown activity

class ObjectFrameEnum(Enum):
    START = 0                  #position / yaw in m / radians relative to starting pose of vehicle 
    CURRENT = 1                #position / yaw in m / radians relative to current pose of vehicle
    GLOBAL = 2                 #position in longitude / latitude, yaw=heading in radians with respect to true north (used in GNSS)
    ABSOLUTE_CARTESIAN = 3     #position / yaw  in m / radians relative to a global cartesian reference frame (used in simulation)

class ObjectPose:
    """
    Represents a hypothetical object position / orientation.
    
    Attributes:
        frame: the frame of reference for the pose. 
        t: if frame=GLOBAL or ABSOLUTE_CARTESIAN, the time in s since the
            epoch, i.e., time.time()  Otherwise, the time since start / current
            in the future, in s
        x: the x position, in the object's frame.  If frame=GLOBAL, this is
            longitude, otherwise forward in m.
        y: the y position, in the object's frame.  If frame=GLOBAL, this is
            latitude, otherwise left in m.
        z: the optional z position, in m and in the object's frame.
        yaw: the optional yaw, in radians and in the object's frame. If
            frame=GLOBAL, this is heading CW from north.  Otherwise, it is
            CCW yaw.
        pitch: the optional pitch, in radians and around left direction in the object's frame
        roll: the optional roll, in radians and around forward direction in the object's frame

    """
    frame : ObjectFrameEnum
    t : float
    x : float
    y : float
    z : Optional[float] = None
    yaw : Optional[float] = None
    pitch : Optional[float] = None
    roll : Optional[float] = None

class AgentState:
    type : AgentEnum
    activity : AgentActivityEnum
    velocity : Tuple[float,float,float]     #estimated velocity in x,y,z, m/s and in agent's local frame
    yaw_rate : float                        #estimated yaw rate, in radians/s
    pose: ObjectPose
    
    def to_dict(self):
        return {
            'type': self.type,
            'activity': self.activity,
            'velocity': self.velocity,
            'yaw_rate': self.yaw_rate,
            'pose': self.pose
        }
# ---------------------------------------------------------

def unpack_pickle(filepath:str)->List[dict]:
    with open(filepath, 'rb') as f:
        data = pickle.load(f)

    for d in data:        
        # detected_agents : List[AgentState]
        # tracking_frames : Dict[AgentEnum, Dict[int, Dict[int, AgentState]]]
        # predicted_trajectories : List[Dict[int,List[AgentState]]]

    # NONE OF THIS CODE IS ACTUALLY VERIFIED AND COULD HAVE BUGS THAT MAKE IT NOT READ THE DATA CORRECTLY
        # unpack detected_agents. 
        detected_agents = []
        for agent in d['detected_agents']:
            new_agent = agent.to_dict()
            detected_agents.append(new_agent)
        d['detected_agents'] = detected_agents

        # unpack tracking_frames
        tracking_frames = {}
        for frame, d_frame in tracking_frames[AgentEnum.PEDESTRIAN]:
            tracking_frames[AgentEnum.PEDESTRIAN][frame] = {}
            for id, agent in d_frame.items():
                new_agent = agent.to_dict()
                tracking_frames[frame][id] = new_agent
        d['tracking_frames'] = tracking_frames

        # unpack predicted_trajectories
        predicted_trajectories = []
        for framenum in range(len(d['predicted_trajectories'])):
            frame = d['predicted_trajectories'][framenum]
            new_frame = []
            for agent in frame:
                new_agent = agent.to_dict()
                new_frame.append(new_agent)
            predicted_trajectories.append(new_frame)
        d['predicted_trajectories'] = predicted_trajectories


    return data

# ---------------------------------------------------------

if __name__ == "__main__":
    all_data = []
    
    dir = os.fsencode(FOLDERPATH)

    for file in os.listdir(dir):
        filename = os.fsdecode(file)
        if filename.endswith(".pkl"):
            data = unpack_pickle(FOLDERPATH + filename)
            all_data += data
        
    print(all_data)

    # save all_data as a json
    with open(FOLDERPATH + "all_data.json", 'w') as f:
        json.dump(all_data, f)

        

