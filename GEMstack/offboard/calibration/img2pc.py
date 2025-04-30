import pyvista as pv
import argparse
import cv2
import numpy as np
from tools.save_cali import load_ex,save_ex,load_in,save_in
from scipy.spatial.transform import Rotation as R
from transform3d import Transform

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

def pc_projection(pc,T:Transform,K,img_shape) -> np.ndarray:
    mask = ~(np.all(pc == 0, axis=1))
    pc = pc[mask]

    pc = T @ pc
    if pc.shape[1] == 4:
        pc = pc[:,:-1]/pc[:,[-1]]

    assert pc.shape[1] == 3
    x,y,z = pc.T
    u = (K[0, 0] * x / z) + K[0, 2]
    v = (K[1, 1] * y / z) + K[1, 2]

    img_h, img_w, _ = img_shape
    valid_pts = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
    return u[valid_pts],v[valid_pts]

def calib(args,pc,img,K,N):
    cpoints = np.array(pick_n_img(img,N)).astype(float)
    print(cpoints)

    lpoints = np.array(pick_n_pc(pc,N))
    print(lpoints)

    success, rvec, tvec = cv2.solvePnP(lpoints, cpoints, K, None)
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
    return c2v

def main():
    parser = argparse.ArgumentParser(description='register image into point cloud using manual feature selection',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-p', '--img_path', type=str, required=True,
                       help='Path to PNG image')
    parser.add_argument('-l', '--pc_path', type=str, required=True,
                       help='Path to lidar NPZ point cloud')
    parser.add_argument('-t', '--pc_transform_path', type=str, required=True,
                       help='Path to yaml file for lidar extrinsics')
    parser.add_argument('-i', '--img_intrinsic_path', type=str, required=True,
                       help='Path to yaml file for image intrinsics')
    parser.add_argument('-o', '--out_path', type=str, required=False,
                       help='Path to output yaml file for image extrinsics')
    parser.add_argument('-n', '--n_features', type=int, required=False, default=8,
                       help='Number of features to select and math')
    parser.add_argument('-u','--no_undistort', action='store_true',
                       help='Whether to use distortion parameters')
    parser.add_argument('-s', '--show', action='store_true',
                       help='Show projected points after calibration')
    

    args = parser.parse_args()

    # Load data
    N = args.n_features
    img = cv2.imread(args.img_path, cv2.IMREAD_UNCHANGED)
    pc = np.load(args.pc_path)['arr_0']
    pc = pc[~np.all(pc == 0, axis=1)] # remove (0,0,0)'s

    if not args.no_undistort:
        K, distort = load_in(args.img_intrinsic_path,mode='matrix',return_distort=True)
        print('applying distortion coeffs', distort)
        h,  w = img.shape[:2]
        newK, roi = cv2.getOptimalNewCameraMatrix(K, distort, (w,h), 1, (w,h))
        img = cv2.undistort(img, K, distort, None, newK)
        K = newK
    else:
        K = load_in(args.img_intrinsic_path,mode='matrix')

    lidar_ex = np.eye(4)
    if args.pc_transform_path:
        lidar_ex = load_ex(args.pc_transform_path,mode='matrix')
        pc = np.pad(pc,((0,0),(0,1)),constant_values=1) @ lidar_ex.T[:,:3]
    
    c2v = calib(args,pc,img,K,N)
    T = np.linalg.inv(c2v)
    print(T)

    if args.show:
        T_lidar_camera = T @ lidar_ex
        u,v = pc_projection(pc,Transform(T_lidar_camera),K,img.shape)
        show_img = img.copy()
        for uu,vv in zip(u.astype(int),v.astype(int)):
            cv2.circle(show_img, (uu, vv), radius=1, color=(0, 0, 255), thickness=-1)
        cv2.imshow("projection", show_img)
        cv2.waitKey(0)

if __name__ == "__main__":
    main()