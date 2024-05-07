import overpass
from typing import Optional
from dataclasses import dataclass

@dataclass
class RoadProperties:
    """Class for keeping track of name, speed limit, number of lanes."""
    name : Optional[str] = None
    max_speed : Optional[float] = None
    lanes : Optional[int] = None

class RoadPropertiesHandler:
    """Finds road properties based on the current latitude and longitude."""
    def __init__(self):
        self.api = overpass.API()
        self.latitude = None
        self.longitude = None
        self.properties = RoadProperties()

    def to_mps(self, speed):
        """Converts the speed to m/s.

        For OSM graph edges, 
        maxspeed = numeric value, followed by the appropriate unit. 
        When the value is in km/h, no unit should be included. 
        Other possible units: mph (miles per hour), knots
        """

        split = speed.split(' ')
        if len(split) == 1: # in km/h
            m = 5 / 18
        else:
            m = 0.44704 if split[1] == 'mph' else 0.514444
        return float(split[0]) * m

    def get_features_within_radius(self, r):
        query = 'way(around:' + str(r) + ',' + str(self.latitude) + ',' + str(self.longitude) + ')'
        response = self.api.get(query)
        return response['features']

    def process_features_within_radius(self, r):
        max_speed = None
        for feature in self.get_features_within_radius(r):
            try:
                self.properties.name = feature['properties']['name']
            except KeyError: # current feature is not a road
                continue

            try:
                max_speed = feature['properties']['maxspeed']
                self.properties.lanes = feature['properties']['lanes']
            except KeyError: # no max speed/ number of lanes available
                pass

            break

        return max_speed

    def update(self, latitude, longitude):
        if self.latitude == latitude and self.longitude == longitude:
            return

        self.latitude = latitude
        self.longitude = longitude

        self.properties = RoadProperties()
        max_speed = None

        for r in range(3, 10): # starting from 3m since standard lane width is 12ft, search radius is 10m
            max_speed = self.process_features_within_radius(r)

            if self.properties.name: # found the road properties
                if max_speed: # is not None
                    self.properties.max_speed = self.to_mps(max_speed)

                break
