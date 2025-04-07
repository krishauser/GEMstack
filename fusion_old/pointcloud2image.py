import cv2
import numpy as np
import open3d as o3d
from SensorFusion import    OAK_RGB_INTRINSIC, OAK_RGB_EXTRINSIC, OAK_RGB_HEIGHT, OAK_RGB_WIDTH, \
                            T_OUSTER_2_BASE_FOOTPRINT, T_LIVOX_2_BASE_FOOTPRINT, T_OAK_2_BASE_FOOTPRINT
import time

def transformPointCloudFrame(pointcloud, T):
    """pointcloud shape should be (N, 3)"""
    assert pointcloud.shape[1] == 3
    ROT = T[:3, :3]
    TRANS = T[:3, 3]
    return pointcloud @ ROT.T + TRANS  # cation : each point represent in row (1, 3)

def visualPointCloudListOpen3D(point_list, colors):
    """each pointcloud shape should be (N, 3)"""
    groups = len(point_list)
    
    pcds = []
    
    for i in range(groups):
        points = point_list[i]
        assert points.shape[1] == 3, "each pointcloud shape should be (N, 3)"
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        pcd.paint_uniform_color(colors[i])
        
        pcds.append(pcd)

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for pcd in pcds:
        vis.add_geometry(pcd)

    render_option = vis.get_render_option()
    render_option.point_size = 1.5

    vis.run()
    vis.destroy_window()
    

def filterPointCloudWithHeight(pointcloud, lower=-np.inf, upper=np.inf):
    assert upper >= lower, f"upper{upper} must > lower{lower}"
    return pointcloud[(pointcloud[:, 2] >= lower) & (pointcloud[:, 2] <= upper)]


def filterPointCloudWithBox(points, x_range=(1, 10), y_range=(-2, 2), z_range=(0.5, 3)):
    """
    x : front & back
    y : left & right
    z : top & bottom
    """
    assert x_range[1] >= x_range[0], "x_range boundary is invalid"
    assert y_range[1] >= y_range[0], "y_range boundary is invalid"
    assert z_range[1] >= z_range[0], "z_range boundary is invalid"

    # 过滤 X, Y, Z 坐标
    mask = (
        (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
        (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
        (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
    )
    
    return points[mask]


def filterPointCloudWithAngle(points, azimuth_range=(-40, 40), elevation_range=(-60, 60)):
    assert azimuth_range[1] >= azimuth_range[0], "azimuth_range boundary is invalid"
    assert elevation_range[1] >= elevation_range[0], "elevation_range boundary is invalid"
    
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    azimuth = np.degrees(np.arctan2(y, x))  # 方位角 Azimuth
    elevation = np.degrees(np.arctan2(z, np.sqrt(x**2 + y**2)))  # 俯仰角 Elevation

    mask = (
        (azimuth >= azimuth_range[0]) & (azimuth <= azimuth_range[1]) &
        (elevation >= elevation_range[0]) & (elevation <= elevation_range[1])
    )

    return points[mask]


def transformPointCloud2Image(pointsWorld, intrinsic, extrinsic):
    pointsCamera = transformPointCloudFrame(pointsWorld, OAK_RGB_EXTRINSIC)  # N*3
    pointsCamera = pointsCamera[:, :2] / pointsCamera[:, 2:3]  # Normalize
    pointsPixel = pointsCamera @ OAK_RGB_INTRINSIC[:2, :2].T + OAK_RGB_INTRINSIC[:2, 2]  # N*2
    # print(pointsPixel[0:3])
    return pointsPixel.astype(int)

def drawPointCloudPixels(pointsPixel, pointsWorld):
    originImg = cv2.imread(r"./data/oak_image.jpg")
    
    canvas = np.zeros((OAK_RGB_HEIGHT, OAK_RGB_WIDTH))
    
    for pt, ptW in zip(pointsPixel, pointsWorld):
        if 0 <= pt[1] < OAK_RGB_HEIGHT and 0 <= pt[0] < OAK_RGB_WIDTH:
            canvas[pt[1], pt[0]] = np.linalg.norm(ptW)
            originImg[pt[1], pt[0]] = [0,0,255]
    
    cv2.imshow("canvas", canvas)
    cv2.imshow("originImg", originImg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()




if __name__ == "__main__":
    # get the lidar data
    ouster_points = np.load("data/ouster_pointcloud.npy")
    print(f"Ouster Point Cloud Shape: {ouster_points.shape}")
    livox_points = np.load("data/livox_pointcloud.npy")
    print(f"Livox Point Cloud Shape: {livox_points.shape}")
    
    # transform to base_footprint frame
    ouster_in_base = transformPointCloudFrame(ouster_points, T_OUSTER_2_BASE_FOOTPRINT)
    livox_in_base = transformPointCloudFrame(livox_points, T_LIVOX_2_BASE_FOOTPRINT)
    
    ouster_in_base = filterPointCloudWithBox(ouster_in_base)
    livox_in_base = filterPointCloudWithBox(livox_in_base)
    
    ouster_in_base = filterPointCloudWithAngle(ouster_in_base)
    livox_in_base = filterPointCloudWithAngle(livox_in_base)
    
    visualPointCloudListOpen3D([ouster_in_base, livox_in_base],
                                colors=[[1,0,0], [0,0,1]])  # red & blue
    
    ouster_pixels = transformPointCloud2Image(ouster_in_base, OAK_RGB_INTRINSIC, OAK_RGB_EXTRINSIC)
    # ouster_pixels = ouster_pixels[::5]
    livox_pixels  = transformPointCloud2Image(livox_in_base , OAK_RGB_INTRINSIC, OAK_RGB_EXTRINSIC)
    # livox_pixels = livox_pixels[::5]
    
    drawPointCloudPixels(livox_pixels, livox_in_base)
    drawPointCloudPixels(ouster_pixels, ouster_in_base)






