import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from pyproj import Transformer
import os
"""
    This script geotags images based on vehicle GNSS data and images collected from the front right and rear right camera.
    It computes the camera poses in the world frame and saves them to a geo.txt file which can be automatically processed by OpenDroneMap for 3D reconstruction.   
"""
# Fixed camera-to-vehicle transform for front-right camera
T_v_to_c_fr = np.array([1.8861563355156226, -0.7733611068168774, 1.6793040225335112]) #form state estimation team
R_v_to_c_fr = R.from_euler('zyx', [45, 20, 0], degrees=True).as_matrix() 

# Fixed camera-to-vehicle transform for rear-right camera
T_v_to_c_rr = np.array([0.11419591502518789, -0.6896311735924415, 1.711181163333824])
R_v_to_c_rr = R.from_euler('zyx', [135, 20, 0], degrees=True).as_matrix()

# Load vehicle pose data
vehicle_df = pd.read_csv("gnss.txt", header=None,
                         names=["lat", "lon", "alt", "yaw", "roll", "pitch"])

# Convert lat/lon to UTM
transformer = Transformer.from_crs("EPSG:4326", "EPSG:32616", always_xy=True)  # UTM Zone 16N for Illinois, change if in another zone.

# Output rows
output_rows = []

# Process each row in the GNSS data
for i, row in vehicle_df.iterrows():
    lat, lon, alt = row["lat"], row["lon"], row["alt"]
    yaw, roll, pitch = row["yaw"], row["roll"], row["pitch"]

    # Position in UTM (meters)
    x, y = transformer.transform(lon, lat)
    z = alt

    # Convert yaw, roll, pitch to rotation matrix (vehicle orientation in world frame)
    R_vehicle = R.from_euler('zxy', [yaw, roll, pitch], degrees=True).as_matrix()

    # Compute front-right camera pose in world frame
    R_camera_fr = R_vehicle @ R_v_to_c_fr
    T_camera_fr = R_vehicle @ T_v_to_c_fr + np.array([x, y, z])
    yaw_fr, pitch_fr, roll_fr = R.from_matrix(R_camera_fr).as_euler('zxy', degrees=True)
    output_rows.append([f"fr_{i:d}.png", T_camera_fr[0], T_camera_fr[1], T_camera_fr[2], yaw_fr, pitch_fr, roll_fr])

    # Compute rear-right camera pose in world frame
    R_camera_rr = R_vehicle @ R_v_to_c_rr
    T_camera_rr = R_vehicle @ T_v_to_c_rr + np.array([x, y, z])
    yaw_rr, pitch_rr, roll_rr = R.from_matrix(R_camera_rr).as_euler('zxy', degrees=True)
    output_rows.append([f"rr_{i:d}.png", T_camera_rr[0], T_camera_rr[1], T_camera_rr[2], yaw_rr, pitch_rr, roll_rr])

# Save to geo.txt
geo_df = pd.DataFrame(output_rows, columns=["image", "x", "y", "z", "yaw", "pitch", "roll"])
geo_df.to_csv("geo_temp.txt", sep=" ", index=False, header=False)

# Add EPSG zone information at the top of the file
epsg_zone = "EPSG:32616"  # Replace with your EPSG zone if different
with open("geo.txt", "w") as geo_file:
    geo_file.write(f"{epsg_zone}\n")  # Write the EPSG zone as the first line
    with open("geo_temp.txt", "r") as temp_file:
        geo_file.write(temp_file.read())  # Append the rest of the data

# Remove the temporary file
os.remove("geo_temp.txt")