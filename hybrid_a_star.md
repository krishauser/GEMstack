# Generating a Hybrid A Star Path

* Run  `GEMstack/onboard/planning/hybrid_a_star.py` with your required start and goal positions. The variables `ox` and `oy` are the obstacles' $x$ and $y$ coordinates.
* Right now, the car's model in the HAS is not synced with the e4 vehicle, we are working on it, you could find the parameters in `GEMstack/onboard/planning/libraries/car.py`, and change them to alter the path being generated, either the resolution of the points being generated or 
* `hybrid_a_star.py` generates a csv file of the path to be followed within `GEMstack/knowledge/routes/hybrid_path.csv`.

# Running in Simulation

* In the main GEMstack folder, run the command `source catkin_ws/devel/setup.bash`.
* Now run `main.py --variant=sim launch/hybrid_a_star.yaml`

# Running on the actual vehicle

* To run on the actual vehicle, drop the variant, run `main.py launch/hybrid_a_star.yaml`

# What needs to be done

* Remove magic constants from `GEMstack/onboard/planning/libraries/car.py` and sync it with `GEMstack/knowledge/vehicle/gem_e4.yaml` and `GEMstack/knowledge/vehicle/gem_e4_geometry.yaml`
* Improve route tracking, maybe adjust curve radius to better suit the car.
* Enable reversing in `GEMstack/onboard/planning/pure_pursuit.py`
