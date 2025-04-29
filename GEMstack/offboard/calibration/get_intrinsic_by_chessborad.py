import cv2
import numpy as np
from tools.save_cali import save_in

import glob
def estimate_intrinsics(image_dir, checkerboard_size):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    w,h = checkerboard_size
    objp = np.zeros((w*h,3), np.float32)
    objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)

    objpoints = [] 
    imgpoints = [] 
    images = glob.glob(image_dir + '*.png')
    for fname in images:
        img = cv2.imread(fname)

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (w,h),None)

        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            img = cv2.drawChessboardCorners(img, (w,h), corners2,ret)
            cv2.imshow('img',img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    return {
        'fx': mtx[0,0],
        'fy': mtx[1,1],
        'cx': mtx[0,2],
        'cy': mtx[1,2],
        'distortion_coeffs': dist[0].tolist()
    }

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 4:
        print(f"Usage: python3 this.py <image_path> <checkerboard_w> <checkerboard_h> [output_path]")
        sys.exit(1)
        
    try:
        image_path = sys.argv[1]
        w = int(sys.argv[2])
        h = int(sys.argv[3])
        params = estimate_intrinsics(image_path, (w,h))
        print("Camera Intrinsics:")
        print(f"Focal Length (fx, fy): {params['fx']:.2f}, {params['fy']:.2f}")
        print(f"Principal Point (cx, cy): {params['cx']:.2f}, {params['cy']:.2f}")
        print(f"Distortion Coefficients: {params['distortion_coeffs']}")
    except Exception as e:
        print(f"Error: {str(e)}")

    if len(sys.argv) == 5:
        out_path = sys.argv[4]
        save_in(out_path, focal=[params['fx'],params['fy']], center=[params['cx'],params['cy']],distort=params['distortion_coeffs'],skew=0)