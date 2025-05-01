import numpy as np
import cv2 as cv
import glob
import os
import argparse

from GEMstack.GEMstack.knowledge.calibration.calib_util import load_in, undistort_image

def main():
    # Collect arguments
    parser = argparse.ArgumentParser(description='Create copies of images with the distortion removed',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-i', '--img_intrinsics_path', type=str, required=True,
                       help='Path to yaml file for image intrinsics')
    parser.add_argument('-f', '--img_folder_path', type=str, required=True,
                       help='Path to folder containing PNG images')
    parser.add_argument('-c', '--camera_name', type=str, required=False,
                       help='Name of the camera used to identify the correct images')
    
    args = parser.parse_args()

    # Get camera intrinsics
    camera_matrix, distortion_coefficients = load_in(args.img_intrinsics_path, return_distort=True)

    # Find image files
    folder = args.img_folder_path
    camera = ''
    if args.camera_name:
        camera = args.camera_name
    image_files = glob.glob(os.path.join(folder, camera + '*.png'))

    for fn in image_files:
        image = cv.imread(fn)
        dst, _ = undistort_image(image, camera_matrix, distortion_coefficients)
        cv.imwrite(fn.replace(camera, camera + "_rect"), dst)


if __name__ == '__main__':
    main()