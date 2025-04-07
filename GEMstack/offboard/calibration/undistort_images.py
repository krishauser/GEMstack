import numpy as np
import cv2 as cv
import glob
import os

camera_matrix = np.array(
    [[1180.753804, 0.000000   , 934.859447],
     [0.000000   , 1177.034929, 607.266974],
     [0.000000   , 0.000000   , 1.000000  ]]
)

distortion_coefficients = np.array([-0.2448506795091457, 0.08202383880344215, 0.0004294271518916802, -0.0012454354245869965, 0.0])

def main(folder, camera):
    image_files = glob.glob(os.path.join(folder, camera + '*.png'))

    for fn in image_files:
        image = cv.imread(fn)
        h,  w = image.shape[:2]
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w,h), 1, (w,h))

        # undistort
        dst = cv.undistort(image, camera_matrix, distortion_coefficients, None, new_camera_matrix)
        cv.imwrite(fn.replace(camera, camera + "_rect"), dst)


if __name__ == '__main__':
    import sys
    folder = 'data'
    camera = 'fr'
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    if len(sys.argv) >= 3:
        camera = int(sys.argv[2])
    main(folder,camera)