import rosbag
from cv_bridge import CvBridge
import cv2
from PIL import Image
import piexif
import numpy as np
import math
import os

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

def convert_gps_to_exif_format(latitude, longitude):
    """Convert GPS coordinates to EXIF format"""
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
    
    return lat_dms, lon_dms, lat_ref, lon_ref

# Create images directory if it doesn't exist
os.makedirs('images', exist_ok=True)

# Open the bag file
bag = rosbag.Bag('fr+gnss+lidar.bag')
bridge = CvBridge()

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
    
    # Convert GPS coordinates to EXIF format
    lat_dms, lon_dms, lat_ref, lon_ref = convert_gps_to_exif_format(lat, lon)
    
    # Create EXIF data
    exif_dict = {
        "GPS": {
            piexif.GPSIFD.GPSLatitudeRef: lat_ref.encode(),
            piexif.GPSIFD.GPSLatitude: lat_dms,
            piexif.GPSIFD.GPSLongitudeRef: lon_ref.encode(),
            piexif.GPSIFD.GPSLongitude: lon_dms
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
