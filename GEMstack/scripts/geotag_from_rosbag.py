import rosbag
from cv_bridge import CvBridge
import cv2
from PIL import Image
import piexif
import numpy as np
import math
import os
from datetime import datetime
import argparse

"""
This script geotags images based on vehicle GNSS data and images collected from any of the four corner cameras.
Functionality is limited to data that is extacted from a rosbag file.
"""

def rad_to_deg(rad):
    """Convert radians to degrees"""
    return rad * 180.0 / math.pi

def interpolate_position(gnss_times, gnss_data, target_time):
    """
    Interpolate position using GNSS data
    """
    # Find the two closest GNSS messages
    idx = np.searchsorted(gnss_times, target_time)
    if idx == 0:
        lat_rad = gnss_data[0].latitude
        lon_rad = gnss_data[0].longitude
        return rad_to_deg(lat_rad), rad_to_deg(lon_rad)
    if idx == len(gnss_times):
        lat_rad = gnss_data[-1].latitude
        lon_rad = gnss_data[-1].longitude
        return rad_to_deg(lat_rad), rad_to_deg(lon_rad)
    
    # Get timestamps and data for interpolation
    t0, t1 = gnss_times[idx-1], gnss_times[idx]
    p0 = gnss_data[idx-1]
    p1 = gnss_data[idx]
    
    # Calculate interpolation factor
    alpha = (target_time - t0) / (t1 - t0)
    
    # Linear interpolation for position (in radians)
    lat_rad = p0.latitude + alpha * (p1.latitude - p0.latitude)
    lon_rad = p0.longitude + alpha * (p1.longitude - p0.longitude)
    
    # Convert to degrees
    return rad_to_deg(lat_rad), rad_to_deg(lon_rad)

def convert_gps_to_exif_format(latitude, longitude, altitude):
    """Convert GPS coordinates and altitude to EXIF format"""
    def decimal_to_dms(decimal):
        decimal = float(decimal)
        degrees = int(decimal)
        minutes = int((decimal - degrees) * 60)
        seconds = ((decimal - degrees) * 60 - minutes) * 60
        return (degrees, 1), (minutes, 1), (int(seconds * 1000), 1000)

    lat_dms = decimal_to_dms(abs(latitude))
    lon_dms = decimal_to_dms(abs(longitude))
    lat_ref = 'N' if latitude >= 0 else 'S'
    lon_ref = 'E' if longitude >= 0 else 'W'
    
    # Convert altitude to EXIF format (rational number)
    alt_ref = 0 if altitude >= 0 else 1  # 0 = above sea level, 1 = below sea level
    altitude = abs(altitude)
    alt_ratio = (int(altitude * 100), 100)  # Multiply by 100 for 2 decimal precision
    
    return lat_dms, lon_dms, lat_ref, lon_ref, alt_ratio, alt_ref

# Open the bag file
parser = argparse.ArgumentParser(
        description='A script to geotag images from a rosbag file using GNSS data.'
    )

parser.add_argument(
        'bag_name', type = str,
        help = 'The name of the rosbag file to process.'
    )
args = parser.parse_args()
bag = rosbag.Bag(args.bag_name, 'r')
bridge = CvBridge()

# Create images directory if it doesn't exist
os.makedirs('images', exist_ok=True)

# First pass: collect and sort GNSS data
print("Collecting GNSS data...")
gnss_times = []
gnss_messages = []

for topic, msg, t in bag.read_messages(topics=['/septentrio_gnss/insnavgeod']):
    gnss_times.append(t.to_sec())
    gnss_messages.append(msg)

# Convert to numpy array for efficient searching
gnss_times = np.array(gnss_times)

print(f"Collected {len(gnss_messages)} GNSS messages")

# Process images with interpolated position data
print("Processing images...")
image_count = 0
for topic, msg, t in bag.read_messages(topics=['/camera_fl/arena_camera_node/image_raw']):
    # Convert ROS image to OpenCV image
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # Explicitly request BGR
    
    # Convert BGR to RGB
    rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    
    # Convert to PIL Image
    pil_img = Image.fromarray(rgb_img)
    
    # Get interpolated position data
    image_time = t.to_sec()
    lat, lon = interpolate_position(gnss_times, gnss_messages, image_time)
    
    # Get altitude from GNSS message (you'll need to interpolate this as well)
    altitude = gnss_messages[np.searchsorted(gnss_times, image_time)].height
    
    # Convert GPS coordinates to EXIF format
    lat_dms, lon_dms, lat_ref, lon_ref, alt_ratio, alt_ref = convert_gps_to_exif_format(lat, lon, altitude)
    
    timestamp = datetime.fromtimestamp(t.to_sec())
    
    exif_dict = {
    "0th": {
        piexif.ImageIFD.Make: "Lucid".encode(),
        piexif.ImageIFD.Model: "Triton 2.3 MP".encode(),
        piexif.ImageIFD.Software: "ROS".encode(),
        piexif.ImageIFD.DateTime: timestamp.strftime("%Y:%m:%d %H:%M:%S").encode(),
        piexif.ImageIFD.ImageDescription: "Captured with ROS and Lucid Triton 2.3 MP".encode(),
        piexif.ImageIFD.XResolution: (msg.width, 1),
        piexif.ImageIFD.YResolution: (msg.height, 1),
        piexif.ImageIFD.ResolutionUnit: 2,  # inches
    },
    "Exif": {
        piexif.ExifIFD.DateTimeOriginal: timestamp.strftime("%Y:%m:%d %H:%M:%S").encode(),
        piexif.ExifIFD.DateTimeDigitized: timestamp.strftime("%Y:%m:%d %H:%M:%S").encode(),
        piexif.ExifIFD.ExposureTime: (1, 100),  # 1/100 second
        piexif.ExifIFD.FNumber: (16, 10),  # f/1.6
        piexif.ExifIFD.ExposureProgram: 1,  # Manual
        piexif.ExifIFD.ISOSpeedRatings: 100,
        piexif.ExifIFD.ExifVersion: b'0230',
        piexif.ExifIFD.ComponentsConfiguration: b'\x01\x02\x03\x00',  # RGB
        piexif.ExifIFD.FocalLength: (16, 1),  # 16mm
        piexif.ExifIFD.ColorSpace: 1,  # sRGB
        piexif.ExifIFD.PixelXDimension: msg.width,
        piexif.ExifIFD.PixelYDimension: msg.height,
        piexif.ExifIFD.ExposureMode: 1,  # Manual exposure
        piexif.ExifIFD.WhiteBalance: 1,  # Manual white balance
        piexif.ExifIFD.SceneCaptureType: 0,  # Standard
    },
    "GPS": {
        piexif.GPSIFD.GPSLatitudeRef: lat_ref.encode(),
        piexif.GPSIFD.GPSLatitude: lat_dms,
        piexif.GPSIFD.GPSLongitudeRef: lon_ref.encode(),
        piexif.GPSIFD.GPSLongitude: lon_dms,
        piexif.GPSIFD.GPSAltitudeRef: alt_ref,
        piexif.GPSIFD.GPSAltitude: alt_ratio,
        piexif.GPSIFD.GPSTimeStamp: tuple(map(lambda x: (int(x), 1), timestamp.strftime("%H:%M:%S").split(":"))),
        piexif.GPSIFD.GPSDateStamp: timestamp.strftime("%Y:%m:%d").encode(),
        piexif.GPSIFD.GPSVersionID: (2, 3, 0, 0),
    }
}
    
    # Convert to bytes
    exif_bytes = piexif.dump(exif_dict)
    
    # Save image with EXIF data
    output_filename = os.path.join('images', f'image_{image_time:.3f}.jpg')
    pil_img.save(
        output_filename,
        'jpeg',
        exif=exif_bytes
    )
    
    image_count += 1

bag.close()
print(f"\nProcessing complete! Saved {image_count} images to the 'images' folder.")
