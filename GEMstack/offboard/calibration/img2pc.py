#%%
import pyvista as pv
import argparse
import cv2
import numpy as np
from tools.save_cali import load_ex,save_ex,load_in,save_in
#%%
def pick_n_img(img,n=4):
    corners = []  # Reset the corners list
    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            corners.append((x, y))
            cv2.circle(param, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow('Image', param)
    
    cv2.imshow('Image', img)
    cv2.setMouseCallback('Image', click_event, img)
    
    while True:
        if len(corners) == n:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return None
    
    cv2.destroyAllWindows()
    
    return corners

#%%
def pick_n_pc(point_cloud,n=4):
    points = []
    def cb(pt,*args):
        points.append(pt)
    while len(points)!=n:
        points = []
        cloud = pv.PolyData(point_cloud)
        plotter = pv.Plotter(notebook=False)
        plotter.camera.position = (-20,0,20)
        plotter.camera.focal_point = (0,0,0)
        plotter.add_mesh(cloud, color='lightblue', point_size=5, render_points_as_spheres=True)
        plotter.enable_point_picking(cb)
        plotter.show()
    return points

#%%
def main():
    parser = argparse.ArgumentParser(description='register image into point cloud using manual feature selection',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--img_path', type=str, required=True,
                       help='Path to PNG image')
    parser.add_argument('--pc_path', type=str, required=True,
                       help='Path to NPZ point cloud file')
    parser.add_argument('--pc_transform_path', type=str, required=False,
                       help='Path to ymal file for lidar calibration')
    parser.add_argument('--img_intrinsics_path', type=str, required=True,
                       help='Path to ymal file for image intrinsics')
    parser.add_argument('--out_path', type=str, required=False,
                       help='Path to output ymal file for image extrinsics')
    parser.add_argument('--n_features', type=int, required=False, default=8,
                       help='number of features to select and math')
    

    args = parser.parse_args()

    # Load data
    N = args.n_features
    img = cv2.imread(args.img_path, cv2.IMREAD_UNCHANGED)
    pc = np.load(args.pc_path)['arr_0']
    pc = pc[~np.all(pc == 0, axis=1)] # remove (0,0,0)'s

    camera_in = load_in(args.img_intrinsics_path,mode='matrix')

    if args.pc_transform_path is not None:
        lidar_ex = load_ex(args.pc_transform_path,mode='matrix')
        pc = np.pad(pc,((0,0),(0,1)),constant_values=1) @ lidar_ex.T[:,:3]
    
    
    cpoints = np.array(pick_n_img(img,N)).astype(float)
    print(cpoints)

    lpoints = np.array(pick_n_pc(pc,N))
    print(lpoints)

    success, rvec, tvec = cv2.solvePnP(lpoints, cpoints, camera_in, None)
    R, _ = cv2.Rodrigues(rvec)

    T=np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    print(T)

    v2c = T
    print('vehicle->camera:',v2c)
    c2v = np.linalg.inv(v2c)
    print('camera->vehicle:',c2v)

    if args.out_path is not None:
        save_ex(args.out_path,matrix=c2v)


if __name__ == "__main__":
    main()