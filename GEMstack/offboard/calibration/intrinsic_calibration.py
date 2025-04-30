import numpy as np
import cv2 as cv
import glob
import os
import argparse

from tools.save_cali import save_in

def main():
    # Collect arguments
    parser = argparse.ArgumentParser(description='calculate intrinsics from checkerboard images',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-f', '--img_folder_path', type=str, required=True,
                       help='Path to folder containing PNG images')
    parser.add_argument('-c', '--camera_name', type=str, required=False,
                       help='Name of the camera used to identify the correct images')
    parser.add_argument('-o', '--out_path', type=str, required=False,
                       help='Path to output ymal file for camera intrinsics')
    
    args = parser.parse_args()

    # Find image files
    folder = args.img_folder_path
    camera = ''
    if args.camera_name:
        camera = args.camera_name
    image_files = glob.glob(os.path.join(folder, camera + '*.png'))

    # The following code is derived from https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(7,5,0)
    objp = np.zeros((6*8,3), np.float32)
    objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
        
    for fname in image_files:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (8,6), None)
    
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
    
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
    
            # Draw and display the corners
            cv.drawChessboardCorners(img, (8,6), corners2, ret)
            cv.imshow('img', img)
            cv.waitKey(500)

    cv.destroyAllWindows()

    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print(repr(mtx))
    print(dist[0])

    if args.out_path:
        save_in(args.out_path, matrix=mtx)


if __name__ == '__main__':
    main()