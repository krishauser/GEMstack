from ..component import Component
import sys
import cv2
import pickle

OUTPUT_FOLDER = "perception_data_collection/"
NITERATIONS_PER_SAVE = 10

class Recording(Component):
    def __init__(self):
        self.zed_image = None

        self.iterations = None

        self.all_data = []

        # save AgentEnum and AgentState



    def initialize(self):
        # tell the vehicle to use image_callback whenever 'front_camera' gets a reading, and it expects images of type cv2.Mat
        self.vehicle_interface.subscribe_sensor('front_camera',self.image_callback,cv2.Mat)
        # # tell the vehicle to use lidar_callback whenever 'top_lidar' gets a reading, and it expects numpy arrays
        # self.vehicle_interface.subscribe_sensor('top_lidar',self.lidar_callback,np.ndarray)
        # # subscribe to the Zed CameraInfo topic
        # self.camera_info_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, self.camera_info_callback)

    def image_callback(self, image : cv2.Mat):
        self.zed_image = image

    # def camera_info_callback(self, info : CameraInfo):
    #     self.camera_info = info

    # def lidar_callback(self, point_cloud: np.ndarray):
    #     self.point_cloud = point_cloud

    def rate(self):
        return 0.5

    def state_inputs(self):
        return ['vehicle', 'tracking_frames', 'predicted_trajectories', 'detected_agents']

    def state_outputs(self):
        return []
    
    def update(self, v, t, p, d):
        if self.zed_image is not None:
            print("No camera image, not saving any of the data. ")
            return
        
        # save the image
        cv2.imwrite(OUTPUT_FOLDER + "image_" + str(self.iterations) + ".png", self.zed_image)

        # save everything else to all_data
        data = {
            'iteration': self.iterations,
            # 'image': self.zed_image,
            'vehicle': v,
            'tracking_frames': t,
            'predicted_trajectories': p,
            'detected_agents': d
        }
        self.all_data.append(data)

        # every NITERATIONS_PER_SAVE, save all_data to a file
        if self.iterations % NITERATIONS_PER_SAVE == 0:
            with open(OUTPUT_FOLDER + "all_data_" + str(self.iterations) + ".pkl", 'wb') as f:
                pickle.dump(self.all_data, f)
            self.all_data = []

        self.iterations += 1
        return 
        
    def cleanup(self):
        # clean up subprocess which runs the model.
        self.model.stdin.close()
        self.model.wait()
        pass