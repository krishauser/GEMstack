roscore

<!-- rosbag play -l ~/Projects/GEMstack/logs/three_02_05_25/vehicle.bag --rate=0.2 -->

<!-- rosbag play -l ~/Projects/GEMstack/logs/parking/FirstParking.bag -->

rosbag play -l ~/Projects/GEMstack/logs/stitch/stitching_front3.bag


roslaunch fusion sensors.launch 

python3 stitch.py




