import sys
import os
sys.path.append(os.getcwd())
from GEMstack.onboard.googlemaps import conversion

Location = input("Where would you like to go? ")
conversion.test_find_place(Location)