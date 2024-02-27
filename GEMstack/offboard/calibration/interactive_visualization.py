# examples/python/visualization/interactive_visualization.py



import copy

from klampt import vis
from klampt.math import so3,se3
from klampt.vis.colorize import colorize
from klampt import PointCloud,Geometry3D
from klampt.io import numpy_convert
from klampt.model.sensing import image_to_points
import cv2
import os
import numpy as np
import math
import time
import open3d as o3d

zed_K = np.array([[527.5779418945312, 0.0, 616.2459716796875],
                [0.0, 527.5779418945312, 359.2155456542969],
                [0.0, 0.0, 1.0]])
zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
zed_w = 1280
zed_h = 720



def demo_crop_geometry():

    print("Demo for manual geometry cropping")

    print(

        "1) Press 'Y' twice to align geometry with negative direction of y-axis"

    )

    print("2) Press 'K' to lock screen and to switch to selection mode")

    print("3) Drag for rectangle selection,")

    print("   or use ctrl + left click for polygon selection")

    print("4) Press 'C' to get a selected geometry")

    print("5) Press 'S' to save the selected geometry")

    print("6) Press 'F' to switch to freeview mode")

    pcd_data = o3d.data.DemoICPPointClouds()

    pcd = o3d.io.read_point_cloud(pcd_data.paths[0])

    o3d.visualization.draw_geometries_with_editing([pcd])



def draw_registration_result(source, target, transformation):

    source_temp = copy.deepcopy(source)

    target_temp = copy.deepcopy(target)

    source_temp.paint_uniform_color([1, 0.706, 0])

    target_temp.paint_uniform_color([0, 0.651, 0.929])

    source_temp.transform(transformation)

    o3d.visualization.draw_geometries([source_temp, target_temp])



def prepare_data():

    pcd_data = o3d.data.DemoICPPointClouds()

    source = o3d.io.read_point_cloud(pcd_data.paths[0])

    target = o3d.io.read_point_cloud(pcd_data.paths[2])

    print("Visualization of two point clouds before manual alignment")

    draw_registration_result(source, target, np.identity(4))

    return source, target



def pick_points(pcd):

    print("")

    print(

        "1) Please pick at least three correspondences using [shift + left click]"

    )

    print("   Press [shift + right click] to undo point picking")

    print("2) After picking points, press 'Q' to close the window")

    vis = o3d.visualization.VisualizerWithEditing()

    vis.create_window()

    vis.add_geometry(pcd)

    vis.run()  # user picks points

    vis.destroy_window()

    print("")

    return vis.get_picked_points()



def register_via_correspondences(source, target, source_points, target_points):

    corr = np.zeros((len(source_points), 2))

    corr[:, 0] = source_points

    corr[:, 1] = target_points

    # estimate rough transformation using correspondences

    print("Compute a rough transform using the correspondences given by user")

    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()

    trans_init = p2p.compute_transformation(source, target,

                                            o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement

    print("Perform point-to-point ICP refinement")

    threshold = 0.03  # 3cm distance threshold

    reg_p2p = o3d.pipelines.registration.registration_icp(

        source, target, threshold, trans_init,

        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    draw_registration_result(source, target, reg_p2p.transformation)
    print(reg_p2p.transformation)



def demo_manual_registration():

    print("Demo for manual ICP")

    source, target = prepare_data()


    # pick points from two point clouds and builds correspondences

    source_points = pick_points(source)

    target_points = pick_points(target)

    assert (len(source_points) >= 3 and len(target_points) >= 3)

    assert (len(source_points) == len(target_points))

    register_via_correspondences(source, target, source_points, target_points)

    print("")
    
def main(folder):
    lidar_xform = se3.identity()


    print("lidar_xform =", lidar_xform)
    zed_xform = (so3.from_ndarray(np.array([[0,0,1],[-1,0,0],[0,-1,0]])),[0,0,0])
    lidar_pattern = os.path.join(folder,"lidar{}.npz")
    color_pattern = os.path.join(folder,"color{}.png")
    depth_pattern = os.path.join(folder,"depth{}.tif")
    data = {}
    idx = 8
    
    arr_compressed = np.load(lidar_pattern.format(idx))
    arr = arr_compressed['arr_0']
    arr_compressed.close()
    pc_lidar = numpy_convert.from_numpy(arr,'PointCloud')
    pc_lidar = colorize(pc_lidar,'z','plasma')
    data['lidar'] = Geometry3D(pc_lidar)

    color = cv2.imread(color_pattern.format(idx))
    depth = cv2.imread(depth_pattern.format(idx),cv2.IMREAD_UNCHANGED)
    depth = depth.astype(np.float32)
    print("depth range",np.min(depth),np.max(depth))
    zed_xfov = 2*np.arctan(zed_w/(2*zed_intrinsics[0]))
    zed_yfov = 2*np.arctan(zed_h/(2*zed_intrinsics[1]))
    print("estimated zed horizontal FOV",math.degrees(zed_xfov),"deg")
    
    pc_zed_data = image_to_points(depth, color, zed_xfov,zed_yfov,depth_scale=4000.0/0xffff, points_format='numpy')

    #test
    pc_vel_data = arr
    pc_vel = o3d.geometry.PointCloud()
    pc_vel.points = o3d.utility.Vector3dVector(pc_vel_data)
    o3d.io.write_point_cloud("vel.pcd", pc_vel)
    pc_zed = o3d.geometry.PointCloud()
    # pc_zed.points = o3d.utility.Vector3dVector(pc_zed_data)
    pc_zed.points = o3d.utility.Vector3dVector(pc_zed_data[:, :3])  # Extract XYZ coordinates
    pc_zed.colors = o3d.utility.Vector3dVector(pc_zed_data[:, 3:])  # Extract and normalize RGB color values
    o3d.io.write_point_cloud("zed.pcd", pc_zed)

    



    print("Load a pcd point cloud, print it, and render it")
    
    o3d.visualization.draw_geometries([pc_zed],
                                    zoom=0.3412,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])




    source_points = pick_points(pc_vel)

    target_points = pick_points(pc_zed)

    assert (len(source_points) >= 3 and len(target_points) >= 3)

    assert (len(source_points) == len(target_points))

    register_via_correspondences(pc_vel, pc_zed, source_points, target_points)




if __name__ == "__main__":

    import sys
    folder = 'data'
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    if len(sys.argv) >= 3:
        calib = sys.argv[2]
        import yaml
        with open(calib,'r') as f:
            config = yaml.load(f,yaml.SafeLoader)
            zed_K = np.array(config['K']).reshape((3,3))
            zed_intrinsics = [zed_K[0,0],zed_K[1,1],zed_K[0,2],zed_K[1,2]]
            zed_w = config['width']
            zed_height = config['height']
    main(folder)

    # demo_crop_geometry()

    # demo_manual_registration()


# examples/python/visualization/interactive_visualization.py


# import numpy as np

# import copy

# import open3d as o3d



# def demo_crop_geometry():

#     print("Demo for manual geometry cropping")

#     print(

#         "1) Press 'Y' twice to align geometry with negative direction of y-axis"

#     )

#     print("2) Press 'K' to lock screen and to switch to selection mode")

#     print("3) Drag for rectangle selection,")

#     print("   or use ctrl + left click for polygon selection")

#     print("4) Press 'C' to get a selected geometry")

#     print("5) Press 'S' to save the selected geometry")

#     print("6) Press 'F' to switch to freeview mode")

#     pcd_data = o3d.data.DemoICPPointClouds()

#     pcd = o3d.io.read_point_cloud(pcd_data.paths[0])

#     o3d.visualization.draw_geometries_with_editing([pcd])



# def draw_registration_result(source, target, transformation):

#     source_temp = copy.deepcopy(source)

#     target_temp = copy.deepcopy(target)

#     source_temp.paint_uniform_color([1, 0.706, 0])

#     target_temp.paint_uniform_color([0, 0.651, 0.929])

#     source_temp.transform(transformation)

#     o3d.visualization.draw_geometries([source_temp, target_temp])



# def prepare_data():

#     pcd_data = o3d.data.DemoICPPointClouds()

#     source = o3d.io.read_point_cloud(pcd_data.paths[0])

#     target = o3d.io.read_point_cloud(pcd_data.paths[2])

#     print("Visualization of two point clouds before manual alignment")

#     draw_registration_result(source, target, np.identity(4))

#     return source, target



# def pick_points(pcd):

#     print("")

#     print(

#         "1) Please pick at least three correspondences using [shift + left click]"

#     )

#     print("   Press [shift + right click] to undo point picking")

#     print("2) After picking points, press 'Q' to close the window")

#     vis = o3d.visualization.VisualizerWithEditing()

#     vis.create_window()

#     vis.add_geometry(pcd)

#     vis.run()  # user picks points

#     vis.destroy_window()

#     print("")

#     return vis.get_picked_points()



# def register_via_correspondences(source, target, source_points, target_points):

#     corr = np.zeros((len(source_points), 2))

#     corr[:, 0] = source_points

#     corr[:, 1] = target_points

#     # estimate rough transformation using correspondences

#     print("Compute a rough transform using the correspondences given by user")

#     p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()

#     trans_init = p2p.compute_transformation(source, target,

#                                             o3d.utility.Vector2iVector(corr))

#     # point-to-point ICP for refinement

#     print("Perform point-to-point ICP refinement")

#     threshold = 0.03  # 3cm distance threshold

#     reg_p2p = o3d.pipelines.registration.registration_icp(

#         source, target, threshold, trans_init,

#         o3d.pipelines.registration.TransformationEstimationPointToPoint())

#     draw_registration_result(source, target, reg_p2p.transformation)



# def demo_manual_registration():

#     print("Demo for manual ICP")

#     source, target = prepare_data()


#     # pick points from two point clouds and builds correspondences

#     source_points = pick_points(source)

#     target_points = pick_points(target)

#     assert (len(source_points) >= 3 and len(target_points) >= 3)

#     assert (len(source_points) == len(target_points))

#     register_via_correspondences(source, target, source_points, target_points)

#     print("")



# if __name__ == "__main__":

#     demo_crop_geometry()

#     demo_manual_registration()