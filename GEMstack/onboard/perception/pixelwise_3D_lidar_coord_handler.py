import cv2
import numpy as np


class PixelWise3DLidarCoordHandler:
    """ Provide pixelwise 3D lidar coordinate relative to the vehicle frame.
        
        Since lidar points are sparse after being projected onto the image frame, 
        we have to do interpolation. This ensures that most image pixels have corresponding 3D coordinates.
        
        Algo overview pic:
        https://github-production-user-asset-6210df.s3.amazonaws.com/22386566/327139371-20dfddd8-be83-418d-8c9a-d921a74895c5.png?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAVCODYLSA53PQK4ZA%2F20240501%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20240501T135905Z&X-Amz-Expires=300&X-Amz-Signature=b423d6fab8a5ece53df0ab70bf07d8c399c76c1513fa1123d8e2d6c3d61c4c36&X-Amz-SignedHeaders=host&actor_id=22386566&key_id=0&repo_id=732167109
    """

    def __init__(self,
                 kernel_size=5,
                 lidar2camera_fn="GEMstack/knowledge/calibration/gem_e4_lidar2oak.txt",
                 intrinsic_fn="GEMstack/knowledge/calibration/gem_e4_intrinsic.txt",
                 lidar2vehicle_fn="GEMstack/knowledge/calibration/gem_e4_lidar2vehicle.txt",
                 xrange=(0, 20),
                 yrange=(-10, 10),
                 zrange=(-3, 1)) -> None:
        """ 
            Args:
                kernel_size: kernel size for interpolation
                lidar2camera_fn: filename of lidar to camera transformation matrix
                intrinsic_fn: filename of camera intrinsic matrix
                lidar2vehicle_fn: filename of lidar to vehicle rear_axle transformation matrix
                xrange: predefined x range for computation. Range values are using lidar frame in meters.
                yrange: predefined y range for computation. Range values are using lidar frame in meters.
                zrange: predefined z range for computation. Range values are using lidar frame in meters.
        """
        self.T_lidar2camera = np.loadtxt(lidar2camera_fn)
        self.T_lidar2vehicle = np.loadtxt(lidar2vehicle_fn)
        self.intrinsic = np.loadtxt(intrinsic_fn)
            
        self.intrinsic = np.concatenate(
            [self.intrinsic, np.zeros((3, 1))], axis=1)
        
        self.kernel_size = kernel_size

        # For saving computation resource, only process lidar points within predefined range.
        # These range values are using lidar frame.
        self.xrange = xrange
        self.yrange = yrange
        self.zrange = zrange

    def interpolate(self, coord_3d_map: np.ndarray):
        def interpolate_single_channel(image, kernel):
            """ helper function """
            kernel = cv2.flip(kernel, flipCode=-1)
            result = cv2.filter2D(
                image, -1, kernel, borderType=cv2.BORDER_CONSTANT)

            # Derive num_elements matrix
            has_value = image > 0
            has_value = has_value.astype(np.uint8)
            num_elements_mat = cv2.filter2D(
                has_value, -1, kernel, borderType=cv2.BORDER_CONSTANT)

            # divide by num of elements correspond to that particular pixel
            num_elements_mat[num_elements_mat == 0] = 1  # avoid divide by zero
            final_result = result / num_elements_mat  # divide element-wise
            return final_result

        # do interpolation
        channels = []
        num_channels = coord_3d_map.shape[-1]
        for i in range(num_channels):
            channel = coord_3d_map[:, :, i]
            kernel = np.ones(
                (self.kernel_size, self.kernel_size), dtype=np.float32)
            result = interpolate_single_channel(channel, kernel)
            channels.append(result)

        interpolate_3D_map = cv2.merge(channels)
        return interpolate_3D_map

    def filter_lidar_by_range(self, point_cloud: np.ndarray):
        xmin, xmax = self.xrange
        ymin, ymax = self.yrange
        zmin, zmax = self.zrange
        idxs = np.where((point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) &
                        (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) &
                        (point_cloud[:, 2] > zmin) & (point_cloud[:, 2] < zmax))
        return point_cloud[idxs]

    def lidar_to_image(self, point_cloud_lidar: np.ndarray):

        homo_point_cloud_lidar = np.hstack(
            (point_cloud_lidar, np.ones((point_cloud_lidar.shape[0], 1))))  # (N, 4)
        pointcloud_pixel = (self.intrinsic @ self.T_lidar2camera @
                            (homo_point_cloud_lidar).T)  # (3, N)
        pointcloud_pixel = pointcloud_pixel.T  # (N, 3)

        # normalize
        pointcloud_pixel[:, 0] /= pointcloud_pixel[:, 2]  # normalize
        pointcloud_pixel[:, 1] /= pointcloud_pixel[:, 2]  # normalize
        point_cloud_image = pointcloud_pixel[:, :2]  # (N, 2)
        return point_cloud_image

    def lidar_to_vehicle(self, point_cloud_lidar: np.ndarray):
        ones = np.ones((point_cloud_lidar.shape[0], 1))
        pcd_homogeneous = np.hstack((point_cloud_lidar, ones))  # (N, 4)
        pointcloud_trans = np.dot(self.T_lidar2vehicle, pcd_homogeneous.T)  # (4, N)
        pointcloud_trans = pointcloud_trans.T  # (N, 4)
        point_cloud_image_world = pointcloud_trans[:, :3]  # (N, 3)
        return point_cloud_image_world

    def get3DCoord(self, image: np.ndarray, point_cloud: np.ndarray):
        """ 
            Return:
                blend_3D_map: containing pixelwise 3D lidar coordinate in vehicle frame with
                    dimensions (H x W x 3), where H and W correspond to the height and width of the image, respectively.
        
            Note:
                if 3d coord of particular pixel == (0, 0, 0), it means we cannot get lidar 3D coord 
                even after interpolation. Users should ignore that particular pixel for subsequent tasks.
        """
        filtered_point_cloud = self.filter_lidar_by_range(point_cloud)

        point_cloud_3D = self.lidar_to_vehicle(filtered_point_cloud)

        point_cloud_image = self.lidar_to_image(filtered_point_cloud)

        # Keep only the lidar points whose projected coordinates lie within the boundary of images
        height, width = image.shape[:2]
        idxs = np.where((point_cloud_image[:, 0] > 0) & (point_cloud_image[:, 0] < width) &
                        (point_cloud_image[:, 1] > 0) & (point_cloud_image[:, 1] < height))[0]

        # initialize coord_3d_map
        coord_3d_map = np.zeros(shape=(height, width, 3),
                                dtype=point_cloud_3D.dtype)
        for idx in idxs:
            u, v = int(point_cloud_image[idx][0]), int(
                point_cloud_image[idx][1])
            coord_3d_map[v][u][:] = point_cloud_3D[idx]

        # interpolate
        interpolate_3D_map = self.interpolate(coord_3d_map)

        # naive blending
        mask = coord_3d_map > 0
        blend_3D_map = coord_3d_map * mask + interpolate_3D_map * (1 - mask)
        
        return blend_3D_map
