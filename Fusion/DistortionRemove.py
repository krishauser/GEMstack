import numpy as np
import cv2
import glob
import os


# front left (distortion & intrinsic)
DISTORT_FRONT_LEFT = np.array([-0.23751890570984993, 0.08452214195986749, -0.00035324203850054794, -0.0003762498910536819, 0.0])
K_FRONT_LEFT = np.array([1230.1440959825889, 0.0, 978.8285075011214, 0.0, 1230.6304235898162, 605.7940335714893, 0.0, 0.0, 1.0]).reshape((3,3))

# front right (distortion & intrinsic)
DISTORT_FRONT_RIGHT = np.array([-0.2448506795091457, 0.08202383880344215, 0.0004294271518916802, -0.0012454354245869965, 0.0])
K_FRONT_RIGHT = np.array([1180.753803927752, 0.0, 934.8594468810093, 0.0, 1177.034929104844, 607.2669740236552, 0.0, 0.0, 1.0]).reshape((3,3))

# rear left (distortion & intrinsic)
DISTORT_REAR_LEFT = np.array([-0.24343640079788503, 0.09125282937288573, 5.21454371097206e-06, -2.2750254391060847e-05, 0.0])
K_REAR_LEFT = np.array([1216.836137433559, 0.0, 955.7290895142494, 0.0, 1216.0457208793944, 599.3150429290699, 0.0, 0.0, 1.0]).reshape((3,3))

# rear right (distortion & intrinsic)
DISTORT_REAR_RIGHT = np.array([-0.24754745389508612, 0.09944435338953724, -3.455420573094537e-05, -0.00022029168609146265, 0.0])
K_REAR_RIGHT = np.array([1210.2941870955246, 0.0, 951.3029561113208, 0.0, 1211.035575904068, 600.5637386855253, 0.0, 0.0, 1.0]).reshape((3,3))


def undistortImage(originImg, cameraMatrix, distCoeffs):
    h,  w = originImg.shape[:2]  # height & width
    # new intrinsic parameter
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix=cameraMatrix, 
                                                        distCoeffs=distCoeffs, 
                                                        imageSize=(w,h), 
                                                        alpha=1, 
                                                        newImgSize=(w,h))  
    
    undistortImg = cv2.undistort(src=originImg, 
                                cameraMatrix=cameraMatrix, 
                                distCoeffs=distCoeffs, 
                                newCameraMatrix=newCameraMatrix)
    return undistortImg, roi


if __name__ == "__main__":
    imgOriginPath = r"./Pics/fr_000.png"
    origin = cv2.imread(imgOriginPath, flags=cv2.IMREAD_COLOR_BGR)
    # origin = cv2.imread(imgOriginPath, flags=cv2.IMREAD_COLOR_RGB)
    undistort, roi = undistortImage(origin, K_FRONT_LEFT, DISTORT_FRONT_LEFT)
    
    x, y, w, h = roi
    undistortCrop = undistort[y:y+h, x:x+w]

    cv2.namedWindow("origin", cv2.WINDOW_NORMAL)  # 允许手动调整窗口
    cv2.resizeWindow("origin", 900, 600)  # 设置窗口大小为 
    cv2.imshow("origin", origin)
    
    cv2.namedWindow("undistort", cv2.WINDOW_NORMAL)  # 允许手动调整窗口
    cv2.resizeWindow("undistort", 900, 600)  # 设置窗口大小为 
    cv2.imshow("undistort", undistort)
    
    cv2.namedWindow("undistortCrop", cv2.WINDOW_NORMAL)  # 允许手动调整窗口
    cv2.resizeWindow("undistortCrop", 900, 600)  # 设置窗口大小为 
    cv2.imshow("undistortCrop", undistortCrop)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()







# camera_matrix = np.array(
#     [[1180.753804, 0.000000   , 934.859447],
#      [0.000000   , 1177.034929, 607.266974],
#      [0.000000   , 0.000000   , 1.000000  ]]
# )

# distortion_coefficients = np.array([-0.2448506795091457, 0.08202383880344215, 0.0004294271518916802, -0.0012454354245869965, 0.0])

# def main(folder, camera):
#     image_files = glob.glob(os.path.join(folder, camera + '*.png'))

#     for fn in image_files:
#         image = cv.imread(fn)
#         h,  w = image.shape[:2]
#         new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w,h), 1, (w,h))

#         # undistort
#         dst = cv.undistort(image, camera_matrix, distortion_coefficients, None, new_camera_matrix)
#         cv.imwrite(fn.replace(camera, camera + "_rect"), dst)


# if __name__ == '__main__':
#     import sys
#     folder = 'data'
#     camera = 'fr'
#     if len(sys.argv) >= 2:
#         folder = sys.argv[1]
#     if len(sys.argv) >= 3:
#         camera = int(sys.argv[2])
#     main(folder,camera)
    
    
    
    
    
    