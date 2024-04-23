import googlemaps
from datetime import datetime
import sys
import os
sys.path.append(os.getcwd())
from GEMstack.state import PhysicalObject,ObjectPose,ObjectFrameEnum
from GEMstack.state.physical_object import _get_frame_chain
from GEMstack.mathutils import transforms
import math
import time
def find_place_coordinates(place_name):
    # Search for places based on the input name
    places_result = gmaps.places(query=place_name)
    if places_result['status'] == 'OK' and len(places_result['results']) > 0:
        # Get the first (best-matched) place from the search results
        place = places_result['results'][0]
        # Extract the place ID
        place_id = place['place_id']
        # Retrieve the details of the place using the place ID
        place_details = gmaps.place(place_id=place_id)
        if place_details['status'] == 'OK':
            # Extract the latitude and longitude coordinates
            lat = place_details['result']['geometry']['location']['lat']
            lng = place_details['result']['geometry']['location']['lng']
             # Create an ObjectPose for the current pose in the global frame
            current_pose_global = ObjectPose(frame=ObjectFrameEnum.GLOBAL, t=time.time(), y=lat, x=lng, yaw=math.radians(90.0))
            # Transform the current pose to the car frame (START frame)
            current_pose_car = current_pose_global.to_frame(ObjectFrameEnum.START, start_pose_abs=start_pose_global)
            # Return the coordinates in the car frame as JSON
            return {'x': current_pose_car.x, 'y': current_pose_car.y, 'yaw': current_pose_car.yaw}
            # # Return the coordinates as a dictionary
            # return {'lat': lat, 'lng': lng}
        else:
            return None
    else:
        return None
if __name__ == "__main__":
    gmaps = googlemaps.Client(key='REPLACE_WITH_YOUR_API_KEY')
    # Define the start pose of the car in the global frame
    start_pose_global = ObjectPose(frame=ObjectFrameEnum.GLOBAL, t=time.time(), y=40.09286250064475, x=-88.23565755734872, yaw=math.radians(90.0))
    place_name = "Highbay"
    coordinates = find_place_coordinates(place_name)
    if coordinates:
        print(f"Coordinates for the best-matched place: {coordinates}")
    else:
        print(f"No coordinates found for '{place_name}'")
    # Geocoding an address
    # geocode_result = gmaps.geocode('201 St Marys Rd, Champaign, IL')
    # print(geocode_result)
    # geocode_result = gmaps.geocode('UIUC IRL - The Highbay Facility')
    # print(geocode_result)
    # # Look up an address with reverse geocoding
    # reverse_geocode_result = gmaps.reverse_geocode((40.714224, -73.961452))
    # # Request directions via public transit
    # now = datetime.now()
    # directions_result = gmaps.directions("UIUC IRL - The Highbay Facility", "Champaign, IL",
    #                                     mode="transit",
    #                                     departure_time=now)
    # # print(directions_result)
    # # Validate an address with address validation
    # addressvalidation_result =  gmaps.addressvalidation(['1600 Amphitheatre Pk'],
    #                                                     regionCode='US',
    #                                                     locality='Mountain View',
    #                                                     enableUspsCass=True)