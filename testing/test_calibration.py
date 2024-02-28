#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

def run_zed_intrinsics():
    import GEMstack.offboard.calibration.zed_intrinsics as zed_intrinsics
    zed_intrinsics.main()

def run_lidar_zed_transform():
    import GEMstack.offboard.calibration.lidar_zed_transform as lidar_zed_transform
    lidar_zed_transform.main()

if __name__ == '__main__':
    print('Options:')
    print('1. Obtain ZED2 intrinsics')
    print('2. Compute Velodyne to ZED2 transform')
    choice = input('Enter choice - ')
    print()

    if choice == '1':
        run_zed_intrinsics()
    elif choice == '2':
        run_lidar_zed_transform()