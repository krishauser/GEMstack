import requests
from datetime import datetime
import sys
import os
# sys.path.append(os.path.join(os.getcwd(), '...'))
# sys.path.append(os.getcwd())
# sys.path.append(os.path.dirname('/home/sam/Documents/GEMstack/GEMstack'))
from ...state import PhysicalObject,ObjectPose,ObjectFrameEnum
from ...state.physical_object import _get_frame_chain

from ...mathutils import transforms
import math
import time

# Replace with your actual API endpoint
API_ENDPOINT = 'http://localhost:5000/find_place'

# Define the start pose of the car in the global frame
start_pose_global = ObjectPose(frame=ObjectFrameEnum.GLOBAL, t=time.time(), y=40.09286250064475, x=-88.23565755734872, yaw=math.radians(90.0))

def test_find_place(end_place):

    response = requests.post(API_ENDPOINT, data={'place_name': end_place})

    assert response.status_code == 200
    data = response.json()
    if 'error' in data:
        print('No places found')
    # print(f"Place: {end_place}")
    # print(f"Error: {data['error']}")
    else:
    # Test case 1: Valid place name
        assert response.status_code == 200
        data = response.json()
        assert 'lat' in data
        assert 'lng' in data
        print(f"Latitude: {data['lat']}")
        print(f"Longitude: {data['lng']}")

        # Convert coordinates to car frame
        current_pose_global = ObjectPose(frame=ObjectFrameEnum.GLOBAL, t=time.time(), y=data['lat'], x=data['lng'], yaw=math.radians(90.0))
        current_pose_car = current_pose_global.to_frame(ObjectFrameEnum.START, start_pose_abs=start_pose_global)

        print(f"Place: {end_place}")
        print(f"Car Frame - X: {current_pose_car.x}, Y: {current_pose_car.y}, Yaw: {current_pose_car.yaw}")



# if __name__ == '__main__':
#     test_find_place()