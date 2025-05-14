from __future__ import print_function

# Python Headers
import os
import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sklearn.cluster import DBSCAN # Added for clustering

image_path = os.path.join(os.path.dirname(__file__), "highbay_image.pgm")


class OccupancyGrid2:

    global image_path

    def __init__(self):

        # Read image in BGR format
        self.map_image = cv2.imread(image_path)

        # Create the cv_bridge object
        self.bridge = CvBridge()
        self.map_image_pub = rospy.Publisher("/motion_image2", Image, queue_size=1)
        self.obs_image_pub = rospy.Publisher("/obstacles", Image, queue_size=1)
        # Subscribe information from sensors
        self.lat = 0
        self.lon = 0
        self.heading = 0

        # # Klampt
        # self.lat_scale = 80
        # self.lon_scale = 80
        # self.lat_start_bt = -1
        # self.lon_start_l = -1

        # Real vehicle
        self.lat_scale    = 0.00062   # 0.0007
        self.lon_scale    = 0.00136  # 0.00131
        self.lat_start_bt = 40.092722  # 40.09269
        self.lon_start_l  = -88.236365 # -88.23628

        # Gazebo
        # self.lat_scale    = 0.000428581
        # self.lon_scale    = -0.000267982
        # self.lat_start_bt = -0.0001 # Needs Tuning
        # self.lon_start_l  = -0.0004

        self.arrow = 40
        self.img_width = 2107
        self.img_height = 1313

    def image_to_gnss(self, pix_x, pix_y):
        """
        Given image coordinates (pix_x, pix_y), return (latitude, longitude).
        Inverse of your:
            pix_x = img_width * (lon - lon_start_l) / lon_scale
            pix_y = img_height - img_height*(lat - lat_start_bt) / lat_scale
        """
        # longitude:
        lon = self.lon_start_l + (pix_x / float(self.img_width)) * self.lon_scale
        lat = (
            self.lat_start_bt
            + ((self.img_height - pix_y) / float(self.img_height)) * self.lat_scale
        )

        return lat, lon

    def image_heading(self, lon_x, lat_y, heading):

        if heading >= 0 and heading < 90:
            angle = np.radians(90 - heading)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle))

        elif heading >= 90 and heading < 180:
            angle = np.radians(heading - 90)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))

        elif heading >= 180 and heading < 270:
            angle = np.radians(270 - heading)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))

        else:
            angle = np.radians(heading - 270)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle))

        return lon_xd, lat_yd
    

    # def draw_obstacle_cones(self, cones_gnss_list):
    #     """
    #     Converts GNSS coordinates of obstacle cones to image coordinates,
    #     draws them on a map, and publishes the map.
    #     - If one cone, draws a 20x20 box around it.
    #     - If multiple cones, draws a bounding rectangle around all valid cones.

    #     Args:
    #         cones_gnss_list (list): A list of tuples, where each tuple is (lon, lat)
    #                             representing an obstacle cone.
    #     """
    #     print(f"[DEBUG] draw_obstacle_cones called with {len(cones_gnss_list)} cones.")
    #     if cones_gnss_list:
    #         print(f"[DEBUG] Input cones_gnss_list: {cones_gnss_list}")

    #     pub_image = np.copy(self.map_image)
    #     print(f"[DEBUG] Copied self.map_image to pub_image. Shape: {pub_image.shape}")

    #     if not cones_gnss_list:
    #         print("[DEBUG] No cones in cones_gnss_list. Publishing original image.")
    #         try:
    #             # Assuming self.obs_image_pub is the correct publisher
    #             self.obs_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
    #             print("[DEBUG] Published image (no cones).")
    #         except CvBridgeError as e:
    #             rospy.logerr("CvBridge Error (no cones): {0}".format(e))
    #             print(f"[DEBUG] CvBridge Error (no cones): {e}")
    #         return

    #     valid_image_points = []
    #     print("[DEBUG] Processing cones for image conversion...")
    #     for i, (lat, lon) in enumerate(cones_gnss_list):
    #         print(f"[DEBUG] Cone {i}: lon={lon}, lat={lat}")
    #         # Exact conversion logic from your provided function
    #         lon_x = int(self.img_width * (lon - self.lon_start_l) / self.lon_scale)
    #         lat_y = int(
    #             self.img_height
    #             - self.img_height * (lat - self.lat_start_bt) / self.lat_scale
    #         )
    #         print(f"[DEBUG] Cone {i}: Converted to x={lon_x}, y={lat_y}")

    #         if 0 <= lon_x < self.img_width and 0 <= lat_y < self.img_height:
    #             valid_image_points.append((lon_x, lat_y))
    #             print(f"[DEBUG] Cone {i} ({lon_x},{lat_y}) is VALID and added to list.")
    #             # Optional: Draw a small circle for each individual valid cone
    #             # cv2.circle(pub_image, (lon_x, lat_y), 3, (0, 255, 255), -1) # Yellow dot
    #         else:
    #             print(f"[DEBUG] Cone {i} ({lon_x},{lat_y}) is INVALID (outside image bounds: w={self.img_width}, h={self.img_height}).")

    #     print(f"[DEBUG] Finished processing cones. Number of valid_image_points: {len(valid_image_points)}")
    #     if valid_image_points:
    #         print(f"[DEBUG] valid_image_points: {valid_image_points}")

    #     if not valid_image_points:
    #         print("[DEBUG] No valid cones within image boundaries. Publishing current image.")
    #         try:
    #             self.obs_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
    #             print("[DEBUG] Published image (no valid cones).")
    #         except CvBridgeError as e:
    #             rospy.logerr("CvBridge Error (no valid cones in image): {0}".format(e))
    #             print(f"[DEBUG] CvBridge Error (no valid cones in image): {e}")
    #         return

    #     if len(valid_image_points) == 1:
    #         print("[DEBUG] Processing a single valid cone.")
    #         x, y = valid_image_points[0]
    #         buffer = 10  # 10px buffer on each side means a 20x20 box
    #         print(f"[DEBUG] Single cone at x={x}, y={y}. Buffer={buffer}")

    #         pt1_x = max(0, x - buffer)
    #         pt1_y = max(0, y - buffer)
    #         pt2_x = min(self.img_width - 1, x + buffer)
    #         pt2_y = min(self.img_height - 1, y + buffer)
    #         print(f"[DEBUG] Single cone box: pt1=({pt1_x},{pt1_y}), pt2=({pt2_x},{pt2_y})")

    #         if pt1_x < pt2_x and pt1_y < pt2_y:
    #             cv2.rectangle(pub_image, (pt1_x, pt1_y), (pt2_x, pt2_y), (255, 100, 100), 2)  # Light Blue box
    #             print("[DEBUG] Drew rectangle for single cone.")
    #         else:
    #             print("[DEBUG] Single cone rectangle has zero or negative area, not drawn.")

    #     else:  # More than one valid cone
    #         print(f"[DEBUG] Processing {len(valid_image_points)} valid cones (multiple).")
    #         all_x_coords = [p[0] for p in valid_image_points]
    #         all_y_coords = [p[1] for p in valid_image_points]

    #         min_x = min(all_x_coords)
    #         max_x = max(all_x_coords)
    #         min_y = min(all_y_coords)
    #         max_y = max(all_y_coords)
    #         print(f"[DEBUG] Multiple cones bounding box: min_x={min_x}, max_x={max_x}, min_y={min_y}, max_y={max_y}")

    #         # Clip the bounding box coordinates to the image dimensions
    #         rect_pt1_x = max(0, min_x)
    #         rect_pt1_y = max(0, min_y)
    #         rect_pt2_x = min(self.img_width - 1, max_x)
    #         rect_pt2_y = min(self.img_height - 1, max_y)
    #         print(f"[DEBUG] Clipped multiple cones bounding box: pt1=({rect_pt1_x},{rect_pt1_y}), pt2=({rect_pt2_x},{rect_pt2_y})")

    #         if rect_pt1_x < rect_pt2_x and rect_pt1_y < rect_pt2_y:
    #             cv2.rectangle(pub_image, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (100, 255, 100), 2)  # Light Green box
    #             print("[DEBUG] Drew rectangle for multiple cones.")
    #         else:
    #             print("[DEBUG] Multiple cones rectangle has zero or negative area, not drawn.")

    #     print("[DEBUG] Preparing to publish final image with drawn obstacles.")
    #     try:
    #         self.obs_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
    #         print("[DEBUG] Successfully published final image.")
    #     except CvBridgeError as e:
    #         rospy.logerr("CvBridge Error (publishing cones): {0}".format(e))
    #         print(f"[DEBUG] CvBridge Error (publishing cones): {e}")

# Ensure rospy and CvBridgeError are available in your environment
# import rospy
# from cv_bridge import CvBridgeError # Or your specific import path

    def draw_obstacle_cones(self, cones_gnss_list, cone_cluster_pixel_threshold=100):
        """
        Converts GNSS coordinates of obstacle cones to image coordinates,
        uses a naive greedy algorithm to cluster nearby cones, draws appropriate
        bounding boxes, and publishes the map.
        - Isolated cones (or clusters of 1) get a 20x20 buffered box.
        - Groups of nearby cones (clusters > 1) get a common bounding box.

        Args:
            cones_gnss_list (list): A list of tuples, where each tuple is (lon, lat)
                                representing an obstacle cone.
            cone_cluster_pixel_threshold (int): The maximum distance in pixels between a point
                                                and a cluster for the point to be added to it.
        """
        rects = []
        # print(f"[DEBUG-NAIVE] draw_obstacle_cones_naive_clustered called with {len(cones_gnss_list)} cones. Threshold: {cone_cluster_pixel_threshold}px")
        if cones_gnss_list:
            print(f"[DEBUG-NAIVE] Input cones_gnss_list: {cones_gnss_list}")

        pub_image = np.copy(self.map_image)
        # print(f"[DEBUG-NAIVE] Copied self.map_image to pub_image. Shape: {pub_image.shape}")

        if not cones_gnss_list:
            # print("[DEBUG-NAIVE] No cones in cones_gnss_list. Publishing original image.")
            try:
                self.obs_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
                # print("[DEBUG-NAIVE] Published image (no cones).")
            except CvBridgeError as e: # Assuming CvBridgeError and rospy are available
                rospy.logerr("CvBridge Error (no cones): {0}".format(e))
                # print(f"[DEBUG-NAIVE] CvBridge Error (no cones): {e}")
            return rects

        valid_image_points = []
        # print("[DEBUG-NAIVE] Processing cones for image conversion...")
        for i, (lat, lon) in enumerate(cones_gnss_list):
            # print(f"[DEBUG-NAIVE] Cone {i}: lon={lon}, lat={lat}")
            lon_x = int(self.img_width * (lon - self.lon_start_l) / self.lon_scale)
            lat_y = int(
                self.img_height
                - self.img_height * (lat - self.lat_start_bt) / self.lat_scale
            )
            # print(f"[DEBUG-NAIVE] Cone {i}: Converted to x={lon_x}, y={lat_y}")

            if 0 <= lon_x < self.img_width and 0 <= lat_y < self.img_height:
                valid_image_points.append((lon_x, lat_y))
                # print(f"[DEBUG-NAIVE] Cone {i} ({lon_x},{lat_y}) is VALID and added to list.")
            else:
                print(f"[DEBUG-NAIVE] Cone {i} ({lon_x},{lat_y}) is INVALID (outside image bounds: w={self.img_width}, h={self.img_height}).")

        # print(f"[DEBUG-NAIVE] Finished processing cones. Number of valid_image_points: {len(valid_image_points)}")
        if valid_image_points:
            print(f"[DEBUG-NAIVE] valid_image_points: {valid_image_points}")

        if not valid_image_points:
            # print("[DEBUG-NAIVE] No valid cones within image boundaries. Publishing current image.")
            try:
                self.obs_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
                # print("[DEBUG-NAIVE] Published image (no valid cones).")
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error (no valid cones in image): {0}".format(e))
                # print(f"[DEBUG-NAIVE] CvBridge Error (no valid cones in image): {e}")
            return rects

        # --- Naive Greedy Clustering and Drawing Logic ---
        if len(valid_image_points) == 1:
            # print("[DEBUG-NAIVE] Only one valid cone. Drawing single cone box.")
            x, y = valid_image_points[0]
            buffer = 10
            pt1_x = max(0, x - buffer)
            pt1_y = max(0, y - buffer)
            pt2_x = min(self.img_width - 1, x + buffer)
            pt2_y = min(self.img_height - 1, y + buffer)
            # print(f"[DEBUG-NAIVE] Single cone box: pt1=({pt1_x},{pt1_y}), pt2=({pt2_x},{pt2_y})")
            if pt1_x < pt2_x and pt1_y < pt2_y:
                cv2.rectangle(pub_image, (pt1_x, pt1_y), (pt2_x, pt2_y), (255, 100, 100), 2)  # Light Blue box
                rects.append((pt1_x, pt1_y, pt2_x, pt2_y))
                # print("[DEBUG-NAIVE] Drew rectangle for the single valid cone.")
            else:
                print("[DEBUG-NAIVE] Single cone rectangle has zero or negative area, not drawn.")
        
        elif len(valid_image_points) > 1:
            # print(f"[DEBUG-NAIVE] Multiple valid cones ({len(valid_image_points)}). Performing naive greedy clustering.")
            
            threshold_sq = cone_cluster_pixel_threshold ** 2 # Compare squared distances
            
            remaining_points_to_cluster = list(valid_image_points) # Make a mutable copy
            final_clusters = [] # List of clusters, where each cluster is a list of points

            while remaining_points_to_cluster:
                # Start a new cluster with the first available point
                seed_point = remaining_points_to_cluster.pop(0)
                
                current_cluster_points_list = [seed_point]
                # Use a set for efficient "already in cluster" checks during BFS-like expansion
                current_cluster_points_set = {seed_point} 
                
                # Queue for BFS-like expansion: points whose neighbors we need to check
                queue = [seed_point] 
                head_of_queue = 0 # Index to simulate dequeuing from list

                while head_of_queue < len(queue):
                    point_to_expand_from = queue[head_of_queue]
                    head_of_queue += 1
                    
                    # Iterate over remaining_points_to_cluster to find neighbors
                    # To avoid modifying list while iterating, build a new list for the *next* iteration of remaining_points_to_cluster
                    next_iteration_remaining_points = []
                    for other_point in remaining_points_to_cluster:
                        dist_sq = (point_to_expand_from[0] - other_point[0])**2 + \
                                (point_to_expand_from[1] - other_point[1])**2
                        
                        if dist_sq <= threshold_sq:
                            # Point is close, add to current cluster and queue if not already processed for this cluster
                            if other_point not in current_cluster_points_set:
                                current_cluster_points_list.append(other_point)
                                current_cluster_points_set.add(other_point)
                                queue.append(other_point)
                            # This point is now claimed by the current_cluster, so it won't be in next_iteration_remaining_points
                        else:
                            # This point is not close to point_to_expand_from, keep it for future clusters
                            next_iteration_remaining_points.append(other_point)
                    
                    remaining_points_to_cluster = next_iteration_remaining_points # Update list for the next point in queue or next seed
                
                final_clusters.append(current_cluster_points_list)
                # print(f"[DEBUG-NAIVE] Formed cluster (size {len(current_cluster_points_list)}): {current_cluster_points_list}")

            # print(f"[DEBUG-NAIVE] Naive clustering resulted in {len(final_clusters)} clusters.")

            # Draw based on final_clusters
            for i, cluster_points in enumerate(final_clusters):
                # print(f"[DEBUG-NAIVE] Drawing cluster {i} with {len(cluster_points)} points.")
                if not cluster_points: continue # Should not happen

                if len(cluster_points) == 1:
                    x, y = cluster_points[0]
                    buffer = 10
                    pt1_x = max(0, x - buffer)
                    pt1_y = max(0, y - buffer)
                    pt2_x = min(self.img_width - 1, x + buffer)
                    pt2_y = min(self.img_height - 1, y + buffer)
                    # print(f"[DEBUG-NAIVE] Cluster {i} (single point) box: pt1=({pt1_x},{pt1_y}), pt2=({pt2_x},{pt2_y})")
                    # if pt1_x < pt2_x and pt1_y < pt2_y:
                    #     cv2.rectangle(pub_image, (pt1_x, pt1_y), (pt2_x, pt2_y), (255, 100, 100), 2)  # Light Blue
                    #     rects.append((pt1_x, pt1_y, pt2_x, pt2_y))
                    # else:
                    #     print(f"[DEBUG-NAIVE] Cluster {i} (single point) box has zero/negative area.")


                    if rect_pt1_x < rect_pt2_x and rect_pt1_y < rect_pt2_y:
                        # It's a rectangle with area
                        cv2.rectangle(pub_image, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6)  # White
                        # print(f"[DEBUG-NAIVE] Drew WHITE rectangle for cluster {i}")
                    elif rect_pt1_x == rect_pt2_x and rect_pt1_y < rect_pt2_y:
                        # It's a vertical line
                        cv2.line(pub_image, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6) # White line
                        # print(f"[DEBUG-NAIVE] Drew WHITE vertical line for cluster {i}")
                    elif rect_pt1_y == rect_pt2_y and rect_pt1_x < rect_pt2_x:
                        # It's a horizontal line
                        cv2.line(pub_image, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6) # White line
                        # print(f"[DEBUG-NAIVE] Drew WHITE horizontal line for cluster {i}")
                    elif rect_pt1_x == rect_pt2_x and rect_pt1_y == rect_pt2_y and len(cluster_points) > 0 :
                        # All points in cluster are the same single point (after clipping or originally)
                        # This case should ideally be caught by len(cluster_points) == 1, but as a fallback for multi-point clusters collapsing to one point:
                        cv2.circle(pub_image, (rect_pt1_x, rect_pt1_y), 6, (255,255,255), -1) # Draw a filled circle
                        # print(f"[DEBUG-NAIVE] Cluster {i} collapsed to a single point, drew circle.")
                        # rects.append((rect_pt1_x, rect_pt1_y, rect_pt2_x, rect_pt2_y))    

                    else:
                        print(f"[DEBUG-NAIVE] Cluster {i} BBox has zero/negative area and is not a simple line. rect_pt1_x={rect_pt1_x}, rect_pt2_x={rect_pt2_x}, rect_pt1_y={rect_pt1_y}, rect_pt2_y={rect_pt2_y}")
        
                else: # Cluster has multiple points
                    all_x_coords = [p[0] for p in cluster_points]
                    all_y_coords = [p[1] for p in cluster_points]

                    min_x = min(all_x_coords)
                    max_x = max(all_x_coords)
                    min_y = min(all_y_coords)
                    max_y = max(all_y_coords)
                    # print(f"[DEBUG-NAIVE] Cluster {i} ({len(cluster_points)} points) raw BBox: min_x={min_x}, max_x={max_x}, min_y={min_y}, max_y={max_y}")

                    rect_pt1_x = max(0, min_x)
                    rect_pt1_y = max(0, min_y)
                    rect_pt2_x = min(self.img_width - 1, max_x)
                    rect_pt2_y = min(self.img_height - 1, max_y)
                    # print(f"[DEBUG-NAIVE] Cluster {i} clipped BBox: pt1=({rect_pt1_x},{rect_pt1_y}), pt2=({rect_pt2_x},{rect_pt2_y})")

                    # if rect_pt1_x < rect_pt2_x and rect_pt1_y < rect_pt2_y:
                    #     cv2.rectangle(pub_image, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 3)  # Light Green
                    #     rects.append((rect_pt1_x, rect_pt1_y, rect_pt2_x, rect_pt2_y))
                    # else:
                    #     print(f"[DEBUG-NAIVE] Cluster {i} BBox has zero/negative area.")
                    # MODIFIED LOGIC TO HANDLE LINES
                    
                
                    rects.append((rect_pt1_x, rect_pt1_y, rect_pt2_x, rect_pt2_y))    
                    if rect_pt1_x < rect_pt2_x and rect_pt1_y < rect_pt2_y:
                        # It's a rectangle with area
                        cv2.rectangle(pub_image, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6)  # White
                        # print(f"[DEBUG-NAIVE] Drew WHITE rectangle for cluster {i}")
                    elif rect_pt1_x == rect_pt2_x and rect_pt1_y < rect_pt2_y:
                        # It's a vertical line
                        cv2.line(pub_image, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6) # White line
                        # print(f"[DEBUG-NAIVE] Drew WHITE vertical line for cluster {i}")
                    elif rect_pt1_y == rect_pt2_y and rect_pt1_x < rect_pt2_x:
                        # It's a horizontal line
                        cv2.line(pub_image, (rect_pt1_x, rect_pt1_y), (rect_pt2_x, rect_pt2_y), (255, 255, 255), 6) # White line
                        # print(f"[DEBUG-NAIVE] Drew WHITE horizontal line for cluster {i}")
                    elif rect_pt1_x == rect_pt2_x and rect_pt1_y == rect_pt2_y and len(cluster_points) > 0 :
                        # All points in cluster are the same single point (after clipping or originally)
                        # This case should ideally be caught by len(cluster_points) == 1, but as a fallback for multi-point clusters collapsing to one point:
                        cv2.circle(pub_image, (rect_pt1_x, rect_pt1_y), 6, (255,255,255), -1) # Draw a filled circle
                        # print(f"[DEBUG-NAIVE] Cluster {i} collapsed to a single point, drew circle.")
                    else:
                        print(f"[DEBUG-NAIVE] Cluster {i} BBox has zero/negative area and is not a simple line. rect_pt1_x={rect_pt1_x}, rect_pt2_x={rect_pt2_x}, rect_pt1_y={rect_pt1_y}, rect_pt2_y={rect_pt2_y}")
        
        
        # --- End of Naive Clustering and Drawing Logic ---

        # print("[DEBUG-NAIVE] Preparing to publish final image with drawn obstacles.")
        try:
            self.obs_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
            print("[DEBUG-NAIVE] RECTS", rects)
            # print("[DEBUG-NAIVE] Successfully published final image.")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error (publishing cones): {0}".format(e))
            # print(f"[DEBUG-NAIVE] CvBridge Error (publishing cones): {e}")

        # print("OCCUPANCY GRID SHAPE", pub_image.shape)
        cv2.imwrite("occupancy_grid_new.png", pub_image)
        return rects
    
    def compute_cone_rectangles(
        self,
        cones_gnss_list,
        cone_cluster_pixel_threshold=100,
        single_buffer=10
    ):
        """
        Returns a list of (x1, y1, x2, y2) white-box coordinates for the given cones.
        """
        # 1) GNSS → image pixels
        valid = []
        for lat, lon in cones_gnss_list:
            x = int(self.img_width  * (lon - self.lon_start_l) / self.lon_scale)
            y = int(self.img_height - self.img_height * (lat - self.lat_start_bt) / self.lat_scale)
            if 0 <= x < self.img_width and 0 <= y < self.img_height:
                valid.append((x, y))

        if not valid:
            return []

        # 2) Single-cone case
        if len(valid) == 1:
            x, y = valid[0]
            x1 = max(0, x - single_buffer)
            y1 = max(0, y - single_buffer)
            x2 = min(self.img_width  - 1, x + single_buffer)
            y2 = min(self.img_height - 1, y + single_buffer)
            return [(x1, y1, x2, y2)] if x1 < x2 and y1 < y2 else []

        # 3) Greedy clustering
        thresh2 = cone_cluster_pixel_threshold ** 2
        pts = valid[:]
        clusters = []
        while pts:
            seed = pts.pop(0)
            cluster = [seed]
            queue = [seed]
            head = 0
            while head < len(queue):
                cx, cy = queue[head]; head += 1
                next_pts = []
                for ox, oy in pts:
                    if (cx - ox)**2 + (cy - oy)**2 <= thresh2:
                        cluster.append((ox, oy))
                        queue.append((ox, oy))
                    else:
                        next_pts.append((ox, oy))
                pts = next_pts
            clusters.append(cluster)

        # 4) Build rectangles
        rects = []
        for cl in clusters:
            if len(cl) == 1:
                x, y = cl[0]
                x1 = max(0, x - single_buffer)
                y1 = max(0, y - single_buffer)
                x2 = min(self.img_width  - 1, x + single_buffer)
                y2 = min(self.img_height - 1, y + single_buffer)
            else:
                xs = [p[0] for p in cl]; ys = [p[1] for p in cl]
                x1, x2 = max(0, min(xs)), min(self.img_width - 1, max(xs))
                y1, y2 = max(0, min(ys)), min(self.img_height - 1, max(ys))
            if x1 < x2 and y1 < y2:
                rects.append((x1, y1, x2, y2))
        return rects
    
    def gnss_to_image_with_heading(self, lon, lat, heading):

        lon_x = int(self.img_width * (lon - self.lon_start_l) / self.lon_scale)
        lat_y = int(
            self.img_height
            - self.img_height * (lat - self.lat_start_bt) / self.lat_scale
        )
        lon_xd, lat_yd = self.image_heading(lon_x, lat_y, heading)

        pub_image = np.copy(self.map_image)

        if (
            lon_x >= 0
            and lon_x <= self.img_width
            and lon_xd >= 0
            and lon_xd <= self.img_width
            and lat_y >= 0
            and lat_y <= self.img_height
            and lat_yd >= 0
            and lat_yd <= self.img_height
        ):
            cv2.arrowedLine(pub_image, (lon_x, lat_y), (lon_xd, lat_yd), (0, 0, 255), 2)
            cv2.circle(pub_image, (lon_x, lat_y), 12, (0, 0, 255), 2)

            ## Debug check if you can convert back to GNSS
            # tmp_lat, tmp_lon = self.image_to_gnss(lon_x, lat_y)
            # print(f'Lat: {self.lat}, Lon: {self.lon}, Converted Lat: {tmp_lat}, Converted Lon: {tmp_lon}')
            ## End Debug check
        try:
            # Convert OpenCV image to ROS image and publish
            self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def gnss_to_image(self, lon, lat):
        lon_x = int(self.img_width * (lon - self.lon_start_l) / self.lon_scale)
        lat_y = int(
            self.img_height
            - self.img_height * (lat - self.lat_start_bt) / self.lat_scale
        )
        print(f"GNSS hiiiii ({lat}, {lon}) → pixel ({lon_x}, {lat_y})")
        pub_image = np.copy(self.map_image)

        if (
            lon_x >= 0
            and lon_x <= self.img_width
            and lat_y >= 0
            and lat_y <= self.img_height
        ):
            cv2.circle(pub_image, (lon_x, lat_y), 12, (0, 0, 255), 2)

        try:
            # Convert OpenCV image to ROS image and publish
            self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def gnss_to_image_coords(self, lon, lat):

        lon_x = int(self.img_width * (lon - self.lon_start_l) / self.lon_scale)
        lat_y = int(
            self.img_height
            - self.img_height * (lat - self.lat_start_bt) / self.lat_scale
        )
        print(f"GNSS ({lat}, {lon}) → pixel ({lon_x}, {lat_y})")
        return lon_x, lat_y