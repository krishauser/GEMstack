# Calibration

## Calibration Data

```
/GEMstack/data
├── calibration_step1
└── calibration_step2
└── camera_intrinsics.txt
└── lidar_vehicle_calibration.txt
└── lidar_zed_calibration.txt
└── zed_vehicle_calibration.txt
```

- `calibration_step1`: All data for lidar-camera calibration (color.png, depth.tif, lidar.npz).
- `calibration_step2`: All data for lidar-vehicle calibration (color.png, depth.tif, lidar.npz).
- `camera_intrinsics`: Camera intrinsics matrix.
- `lidar_vehicle_calibration.txt`: Lidar to vehicle transformation matrix. 
- `lidar_zed_calibration.txt`: Lidar to zed camera transformation matrix. 
- `zed_vehicle_calibration.txt`: Zed camera to vehicle transformation matrix. 

## HW 3 Step 1 (Relative calbration of lidar and front Zed camera)

### Camera Intrinsics

Run `get_zed_intrinsics.py` in the GEMe2 to obatin the camera intrinsics matrix `K`.

### ICP Registration Method

Run `python3 GEMstack/offboard/calibration/klampt_lidar_zed_show_icp.py data/calibration_step1` to output the lidar-camera transformation matrix. 

## HW 3 Step 2 (Absolute calibration of vehicle height, rear axle center, and sensors)

1. Manually measure the height of rear axle center and displacement along the x-axis.

2. Use `pc-to-ground.py`, manually select lidar points that are on the ground and obtain the plane equation. We can derive the height and roll of lidar. The ground equation is  $-0.03852875x-0.01000802y+0.99920737z+2.1135=0$.

<img width="941" alt="image" src="https://github.com/krishauser/GEMstack/blob/s2024_group4/GEMstack/offboard/calibration/asset/image-20240305202142389.png">

3. Use `centerline.py`, manually choose the lidar lines that correspond to the object that is on the centerline. Assuming the center of the object is on the centerline, we can calculate the yaw. The center is $(8.077,0.159,-0.420)$.Or we can calculate the yaw from plane equation. The plane equation is $0.998x - 0.0025y + 0.0614z -8.03562 = 0.$

<img width="941" alt="image" src="https://github.com/krishauser/GEMstack/blob/s2024_group4/GEMstack/offboard/calibration/asset/image-20240305202212681.png">

4. The resultant transformation is stored in the configuration file.
