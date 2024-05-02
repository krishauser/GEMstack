from ...state import AllState,VehicleState,PhysicalObject,ObjectPose,ObjectFrameEnum
from ...utils import settings
from ...mathutils import collisions
from ..interface.gem import GEMInterface
from ..component import Component
from .pixelwise_3D_lidar_coord_handler import PixelWise3DLidarCoordHandler
from .point_cloud_manipulation import transform_point_cloud

import numpy as np
import math
from sklearn.cluster import DBSCAN

pcd_manip_v = 1 # version 2 uses the PixelWise3DLidarCoordHandler

class ObjectDetector():
    def __init__(self, vehicle : VehicleState, camera_image, lidar_point_cloud, detector):
        self.vehicle = vehicle
        
        self.camera_image = camera_image
        self.lidar_point_cloud = lidar_point_cloud
        
        self.detector = detector

    def box_to_object(self, bbox_xywh):
        """Creates a PhysicalObject from a (x,y,w,h) bounding box."""

        x, y, w, h = bbox_xywh
        # print('Bbox: [{0:.3f}, {1:.3f}, {2:.3f}, {3:.3f}]'.format(x, y, w, h))

        if pcd_manip_v == 1:
            """
            Uses the image, the camera intrinsics, the lidar point cloud, 
            and the calibrated camera / lidar poses to get a good estimate 
            of the object's pose (in vehicle frame) and dimensions.
            """

            # Obtain point clouds in image frame and vehicle frame
            pcd_image_pixels, pcd_vehicle_frame = transform_point_cloud(
                self.lidar_point_cloud, self.camera_image.shape[0], self.camera_image.shape[1]
            )

            # Select points corresponding to the bbox
            indices = [i for i in range(len(pcd_image_pixels)) if 
                        (x - w/2) <= pcd_image_pixels[i][0] <= (x + w/2) and 
                        (y - h/2) <= pcd_image_pixels[i][1] <= (y + h/2)]
            points = [pcd_vehicle_frame[idx] for idx in indices]   # in vehicle frame
            
            # Spatial clustering
            model = DBSCAN(eps=0.2, min_samples=5)
            cluster_ids = model.fit_predict(points)

            unique_cluster_ids, cluster_sizes = np.unique(cluster_ids, return_counts=True)
            if len(unique_cluster_ids) != 1:
                points = [points[i] for i in range(len(points)) 
                          if cluster_ids[i] == unique_cluster_ids[np.argmax(cluster_sizes)]]

        else:
            x = round(bbox_xywh[0])
            y = round(bbox_xywh[1])
            
            # Obtain 3d world coordinates for all pixels in the image
            handler = PixelWise3DLidarCoordHandler()
            all_points = handler.get3DCoord(self.camera_image, self.lidar_point_cloud) # img ht x img width x 3

            # Filter points in bbox
            points = []
            for i in range(x - math.ceil(w/2), x + math.ceil(w/2)):
                for j in range(y - math.ceil(h/2), y + math.ceil(h/2)):
                    # if 3d coord is (0,0,0) ==> pixel is too close to the car (no lidar data available)
                    if any(all_points[j][i]):
                        points.append(all_points[j][i])

            if not points: # no lidar data available for object 
                return None

        # Estimate center and dimensions
        center = np.mean(points, axis=0)
        dimensions = np.max(points, axis=0) - np.min(points, axis=0)
        
        # For a PhysicalObject, 
        #   origin is at the object's center in the x-y plane and at the bottom in the z axis
        pose = ObjectPose(t=0, x=center[0], y=center[1], z=center[2] - dimensions[2]/2, 
                          yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
        
        return PhysicalObject(pose=pose, dimensions=tuple(dimensions), outline=None)

    def detect_objects(self, class_ids):
        detection_result = self.detector(self.camera_image, classes=class_ids, verbose=False)
        bbox_locations = detection_result[0].boxes.xywh.tolist()
        bbox_classes = detection_result[0].boxes.cls.tolist()

        detected_objects = []
        for i in range(len(bbox_locations)):
            detected_object = self.box_to_object(bbox_locations[i])
            
            if detected_object is not None:
                detected_objects.append(detected_object)
        
        return detected_objects, bbox_classes
