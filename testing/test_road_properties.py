import sys
import os
import timeit
sys.path.append(os.getcwd()) # to import GEMstack from top level directory

from GEMstack.onboard.planning.road_properties import RoadPropertiesHandler

road = RoadPropertiesHandler()

def get_properties(latitude, longitude):
    t1 = timeit.default_timer()

    road.update(latitude, longitude)

    t2 = timeit.default_timer()
    print('  Update Time:', '{:.3f}'.format(t2-t1), 'seconds')

    print('  Name:', road.properties.name)
    print('  Speed:', road.properties.max_speed)
    print('  Lanes:', road.properties.lanes)

    t3 = timeit.default_timer()
    print('  Access Time:', '{:.6f}'.format(t3-t2), 'seconds\n')
    
def test_sample_coordinates():
    coords = [] # latitude, longitude, annotation
    coords.append([40.114675, -88.228936, 'S Wright St, next to ECE dept'])
    coords.append([40.113527, -88.224842, 'W Stoughton St, next to Siebel center'])
    coords.append([40.113984, -88.224009, 'N Goodwin Ave, next to Siebel center'])
    coords.append([40.110305, -88.227873, 'near the S Wright & Green St intersection'])
    coords.append([40.094494, -88.237436, 'St Marys Rd, next to I Hotel'])
    coords.append([40.112706, -88.228251, 'W Springfield Ave, next to CIF'])
    coords.append([40.116376, -88.227432, 'W University Ave, next to Beckman Institute'])

    for coord in coords:
        print('Latitude:', coord[0])
        print('Longitude:', coord[1])
        print('Where:', coord[2])
        get_properties(coord[0], coord[1])

def test_input_coordinates(latitude, longitude):
    latitude = float(latitude)
    longitude = float(longitude)

    print('Latitude:', latitude)
    print('Longitude:', longitude)
    
    get_properties(latitude, longitude)

if __name__ == '__main__':
    # optional args: latitude, longitude

    if len(sys.argv) == 1:
        test_sample_coordinates()
    else:
        test_input_coordinates(sys.argv[1], sys.argv[2])
