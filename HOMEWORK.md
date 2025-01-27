# CS 588 Phase 1 Homework 

Due date: 2/24 

**Overall objective**: work together in large teams to create a nontrivial integrated behavior on the vehicle. The target behavior will be to drive along a predefined track, and slowing down for pedestrians that would otherwise be hit if the vehicle continues at normal speed. The vehicle should stop when necessary.

**Skills**: Familiarization with GEMstack repo, Git branches and pull requests, lab and equipment logistics, inter-team communication.

**On-board development**: If you are working on the GEM vehicle, your work will go into the `cs588_groupX/` folder.  We recommend that before you start, you
a) Ensure that your GEMstack repository is on the correct branch.  cd into `cs588_groupX/GEMstack` and run `git status`.  It should show that you are on the `s2025_groupX` branch.
b) Get the latest updates to the course stack.  To do so, run `git pull` and `git merge s2025`.

When you are done, commit your development of this component and launch files to your group's branch.  **DO NOT** set the global git authentication on the vehicle to your account.  The easiest way to do this is:
a) Log onto Github under your account name. [Create a Github Personal Access Token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens#creating-a-personal-access-token-classic).  To reduce the chance of mischief, choose the "Only select repositories" option and set the access token just to push to this repository.  Open the "Repository permissions" and change the "Contents" access to "Read and write"
b) When your token is generated, copy the token and keep it somewhere safe.
c) Commit your changes to your branch.
d) Run "git push".  In "Username" put your Github username, and in "Password" use the token that you copied in step b).  This should now be reflected in your branch.

When you are creating a pull request to the `s2025_teamX` branch of GEMstack, you can do this through the Github webpage.  You can also use the hub command line tool: https://hub.github.com/.  Ensure that this shows up in the `s2025_teamX` branch's Pull Requests tab.

**Quality-of-life tips**
- By default, the log folder is shown after each run.  If this is annoying, change the launch file to read `after: show_log_folder: False`.
- Each component can generate CSV files that collect fast streaming data for later analysis.  Use `self.debug(item,value)` or `self.debug_event(label)` in your component's initialize(), update(), or cleanup() functions.  These will be stored in LOG_FOLDER/COMPONENT_debug.csv.
- Find all magic constants in your code.  Move them into the `GEMstack/knowledge/defaults/current.yaml` file under appropriate sub-keys.


**To submit**: ensure your group's work is merged into the `s2025_teamA` or `s2025_teamB` branches (as appropriate) with appropriately logged PRs. We will review your team's branch history.  Your group will submit some evidence of its component working, and if successful  your team will submit an integrated video of the behavior working.  These may be posted to Slack or Clickup.


## Group Leads

-   Decide on a good day of the week and time for group meetings.
-	Discuss and decide on Github pull request procedures for your team's branches. 
-	Discuss and decide on project management and communication features for your team (e.g., Slack / Clickup).
-	Lead your group and team with clear expectations to prepare work in time for integration. Don't leave this to the last minute! Suggest establishing an internal deadline of 2/17 for all technical pieces.
-   Keep track of team members' efforts, especially if their contributions are not apparent through branch commit history.  At the end of this phase you will be reporting on your members' contributions, so if they are not listed in PRs and you have little to say about their work, they will receive low individual scores for this assignment.  Note that they will be reporting on your leadership as well, so remember to treat your group fairly.
-	Gather sufficient evidence (e.g., videos of partial system tests or simulation tests, or a report with text and figures) that your group has succeeded in its objectives.
-	Decide which group on your team will test the integrated behavior. Have them report results, and capture a video of successful runs.


## Control & Dynamics Group
-   Starter code is provided in the `homework/` folder in your group's branch.
-	All members must complete safety lookout training; at least two must complete safety driver training.

Part 1:

1. Flash a distress signal to the vehicle via ROS. Modify the `blink.py` program to print messages from some of the "/pacmod/parsed_tx/X" topics (at least 2).  For each topic, you will need to create a ROS subscriber.  Read the documentation on https://github.com/astuff/pacmod2 to figure out which message types to use and extract printable data from these messages.
   
    Now, create a publisher for the "/pacmod/as_rx/turn_cmd" topic.  You will then flash a "distress signal" corresponding to left turn signal for 2 seconds, right turn signal for 2s, and then turn signals off for 2s.  This will then repeat forever until you press Ctrl+C.

    To run your code on the vehicle, you will launch the sensors and drive-by-wire system, then enable Pacmod control via the joystick.  (Please do not try to operate the vehicle with the joystick!)  Then, run your program with `python3 blink.py`.  After you are done, you should disable Pacmod control via the joystick.

2. Adapt `blink.py` to run the behavior in the GEMstack executor.  Copy blink_launch.yaml to `GEMstack/launch`.  Copy `blink_component.py` to `GEMstack/onboard/planning/` and make your edits there.

    Replicate your sensor printing and distress signal code in the BlinkDistress class in blink_component.py.  You will need to adapt your code from using ROS subscribers / publishers to using the `GEMVehicleInterface` and `GEMVehicleReading` objects provided in the template code.  Opening the GEMstack folder in the VSCode IDE will help you find the documentation for these classes.

    Run your code in simulation first.  (If you are running on the vehicle, first open one terminal window and run `roscore`. Keep this running.)  Then, in the GEMstack folder, run `python3 main.py --variant=sim launch/blink_launch.yaml`.  You should see your blinking sequence in a matplotlib window.  Use Ctrl+C to quit.

    Now, run your code on the real vehicle.  Enable Pacmod control as before, then run `python3 main.py launch/blink_launch.yaml`.

3. Adapt your code so the distress signal runs on GEMstack as a `signaling` component, and blinks the distress signal only when the `AllState.intent.intent=HALTING` flag is set.  You will need to modify the computation graph in `GEMstack/knowledge/defaults` to add the `signaling` component, and modify launch files so your class is no longer listed under `trajectory_tracking`.
4. Use Git to create a pull request, and have your PR approved to the `s2025_teamX` branch.

Part 2: 
1. Tune the low-level controller (currently `PurePursuit`) to track the trajectory `AllState.trajectory` produced by the Planning group as accurately as possible. You may tune steering, acceleration/braking, the tracking strategies, or all three. 
2. Decide on an appropriate tracking quality metric with the Infra team.
3. Report on what you tuned and how the tracking accuracy metric was improved.
4. Move all magic constants to an appropriate settings file, and have your pull request approved.
5. Integrate as necessary.


## State estimation & Calibration Group

-	Have at least one team member complete safety lookout & driver training.

Part 1:

1. You will be using both the Oak RGB-D camera and the Ouster top lidar to estimate the 3D positions of pedestrians relative to the vehicle frame.  Because the stereo depth estimate is not as accurate as the lidar, we will be using the image to detect objects and the lidar to measure their 3D extent.  The first step of this process is to associate points in the camera with points in the lidar, which requires an understanding of projection and relative pose (extrinsics) calibration.  
2. You will first need to capture a calibration dataset consisting of paired images and lidar scans.  Write code to capture paired scans and take several snapshots of interesting scenes.  You should save images as standard image files, and save lidar scans as numpy arrays or PCD files.  Store these to a directory named GEMstack/data/SOMETHING where "SOMETHING" is a descriptive name of your dataset.  In the main GEMstack folder we had a GEM e2 program `python3 GEMstack/offboard/calibration/capture_lidar_zed.py data/SOMETHING` that could capture these files.  Create a similar program `capture_ouster_oak.py` that reads from the appropriate ROS topics. 
3.  Use the `CameraInfo` message to obtain the intrinsics of the camera (http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html) from the `/oak/rgb/camera_info` ROS topic.  You may use the image_geometry module (https://docs.ros.org/en/api/image_geometry/html/python/) to project points in the camera frame to the image.  A camera frame is defined with the z axis pointing forward, x pointing right, and y pointing down.  The important values of this message are the fx, fy, cx ,and cy intrinsic parameter terms.  You can store these values to disk or read them live in your code.
4. We do not have the relative transform `T_toplidar^frontcamera` that converts points from the top lidar (Ouster) frame to the Oak frame.  Create a way to calibrate this transform.  Specifically, if you have a dataset of paired image, point cloud pairs `(image^t,point_cloud^t)`, you should observe that the points belonging to some object in the point cloud are mapped directly to those pixels in the image.  Specifically, if `image_pt` is an image pixel belonging to point pt in the point cloud, then we should observe the relation "image_pt = project(T_toplidar^frontcamera * pt, oak_intrinsics)"

    To obtain this matrix you may use manual calibration if you need to, or you could use a more sophisticated method. 

    For manual calibration, you can modify the GEM e2 code `python3 GEMstack/offboard/calibration/klampt_show_lidar_zed.py data/SOMETHING` to visualize the stored point clouds from the front camera and the lidar.  You can see why the forward depth estimates are quite poor.

    Automated methods could use a detection method to obtain a distinctive shape like a sphere or a square (e.g., a face of the foam cubes) in the Zed image, paired with the same shape in the lidar point cloud.  An optimization would ensure that all lidar points belonging to the shape are projected to its projection in the image.  You can also match the stereo and lidar point clouds using a dynamically-associated registration method like ICP.


Part 2:

1. Now we will perform absolute calibration of vehicle height, rear axle center, and sensors, i.e., the vehicle frame.  The vehicle frame is centered at the rear axle center, with x pointing forward, y pointing to the left, and z pointing up.  We want the transforms `T_frontcamera^vehicle` and `T_toplidar^vehicle`.

    - A flat ground and walls are good references for the pitch/roll of the sensors.  Using the lidar point cloud, calibrate the rotation component of this transform.  Also, use the height of the ground to determine the z above ground.
    - Using distinctive shapes (e.g., the foam cubes), create a dataset in which you set up objects on the centerline of the vehicle.  Use these to calibrate the yaw and the horizontal offset of the lidar.
    - Measure the height of the rear axle over the ground.  Use this to figure out the z above the rear axle center.

2. Combine all of these measurements to establish `T_toplidar^vehicle`.  Use your result in Step 1 to compute `T_frontcamera^vehicle` as well.  Store these calibrations into the files referenced by `GEMstack/knowledge/calibration/gem_e4.yaml`.  Name your calibrations meaningfully -- specifying which vehicle and what type of calibration you are performing -- and include the time and data of the calibration in your files.
3. Put your offline calibration code in the `GEMstack/offboard/calibration/` directory and write a README with a description of the method that you used, and instructions about how to use it.  Describe the results of your calibration steps 1 and 2 in this file.

    DO NOT commit data files used in calibration.  Put your datasets in the `data` directory, named appropriately. Files in this directory will not be committed to the repo.  

4. Use Git to create a pull request to the `s2025_teamX` branch, and have your PR approved.
5. Integrate as necessary.


## Perception Group

-   Starter code is provided in the `homework/` folder in your group's branch.
-	Have at least one team member complete safety lookout & driver training.
-   Many of your team members should plan to work on the vehicle, at least in the early stages of the assignment. Plan to coordinate with the Infra team to enable a method for working offline.

Part 1:
1. Use an object detector to identify a pedestrian from the front camera.  Implement the function `detect_people(img)` in `person_detector.py` to return a list of bounding boxes of people in an image.  This is a function that inputs OpenCV images and returns a list of (x,y,w,h) image boxes that contain people.  To run it, call `person_detector.py IMAGE` where IMAGE is an image file.  You may download images from the internet or take your own.  If you have a webcam, you can run `person_detector.py webcam` to run the live detector.

    I suggest using the YOLOv8 package (https://github.com/ultralytics/ultralytics) and choosing one of its pretrained models, such as `yolov8n.pt`.  Follow the basic tutorials for running the detector.  The ID of people is 0, and you can examine the Result object to extract out the boxes that have that ID.
2. Refactor your detector to work with GEMstack. Move your detector logic into a GEMstack Component defined in `pedestrian_detection.py`.  The provided code will turn your boxes into a dictionary of `AgentState` objects.  Read the documentation of the `AgentState` class to see what data should be stored there.  For now, you will put temporary values for most of these fields.

    Specifically you should:

    1. Implement the `PedestrianDetector2D` component to call your detection function in `image_callback(img)`. Your model should be loaded once in the component's `initialize()` method.
    2. Move your pretrained model to `GEMstack/knowledge/detection`.  
    3. Move the Python code to `GEMstack/onboard/perception` where the launch file can find them. 
    4. Make sure that your code can find the pretrained model in the `knowledge` directory.  
    5. Using the detector-only variant of the launch file `python3 main.py --variant=detector_only pedestrian_detection.yaml`, test your code on the real vehicle.  You will need to run "source ~/demo_ws/devel/setup.bash" and "roslaunch basic_launch sensor_init.launch"  in another terminal, and run the same "source ..." command before running GEMstack code.
    6. Have a group member(s) walk in front of the Zed camera and observe the printouts.
    7. Finally, tune the rate at which you run your component to obtain the highest framerate possible.

3. Use Git to create a pull request to `s2025_teamX`, and have your PR approved.

Part 2:
1. Fuse detections with depth (`front_depth` sensor), multiple cameras, and/or LIDAR (`top_lidar` sensor) to reduce errors and place agents in their 3D locations in the scene. Use the sensor calibration from the Calibration group.  Look at the lidar readings near the center of the detection, and use those to estimate a region of interest in the point cloud.  Then, fit the center and dimensions of the object to the points in the point cloud that you associate with the pedestrian.  Finally, put the estimated center and dimensions of the pedestrian into the "pose" and "dimensions" attributes, making sure to respect the vehicle's coordinate convention.
2. If the Infra team has worked quickly enough, you should be able to work offline, replaying from ROS bags. It is a good idea to coordinate with them.
3. Provide your results in a `PedestrianDetector` component and provide this to the Planning group.

Part 3: 
1. Keep track of `AgentState` objects as the vehicle moves and when they disappear out of the field of view.
2. Track pedestrians over time to fill out the "velocity" attribute and to give them relatively consistent names over time.   
   - number pedestrian IDs as "ped1", "ped2", etc. so that if you detect that a single pedestrian has moved across your field of view, you consistently output "ped1" as the agent's ID.
   - keep track of the detected pedestrians on the prior frame to estimate the velocity of each pedestrian.  Make sure to store 3D previous detections in the START frame.  For each current detection, examine whether the box of the new detection overlaps any prior detection.  If so, this begins a "track" detection in which you a) keep the last ID, and b) estimate the pedestrian's velocity in the CURRENT frame.  The reported velocity should NOT be relative to the vehicle's velocity.  If this is a new pedestrian, increment the pedestrian ID and output zero velocity for this frame.

    If you wish to use more sophisticated trackers, e.g., Hungarian algorithm, velocity prediction, etc, you may do so.

3. Make sure this works while the vehicle is moving. For example, if the pedestrian is staying still but the vehicle is moving, their reported velocity remains relatively close to 0. Provide these results to the Planning group.
4. Integrate as necessary.


## Planning Group

-	Have at least one team member complete safety lookout & driver training.

Part 1: 
1. We will introduce two new components that operate on the output of a `PedestrianDetector2D` / `PedestrianDetector` model produced by the perception team:

    - Logic in the `PedestrianYielder` component in `pedestrian_yield_logic.py`. 
    - A planner in the `YieldTrajectoryPlanner` component in `longitudinal_planning.py`. 

    Move these files to `GEMstack/onboard/planning` where launch files can find them.

3. For the planning logic, you will implement a braking strategy in `longitudinal_plan_brake()`, and implement a trapezoidal velocity profile strategy in `longitudinal_plan()`.

    You will first do your development using a unit test script, then in simulation.  Run `test_longitudinal_planning.py` from the main GEMstack folder to plot the results of your methods.  Using the launch file `python3 main.py --variant=fake_sim pedestrian_detection.yaml`, you will receive perfect knowledge about the pedestrian.  Test to make sure this produces reasonable behavior in the simulation. It should slow and stop at a safe distance when a pedestrian is detected, and then resume slowly when the pedestrian disappears.
4. Run on the vehicle using `--variant fake_real`.  This will use a "fake" detector that will just report pedestrians at some point.  Work with the Control and Dynamics team to ensure that reasonable behavior results.
5. Use Git to create a pull request, and have your PR approved.

Part 2:
1. Use collision detection to determine the distance to collision and use that to determine whether to begin to brake.

   - Determine the lookahead distance that you would need to avoid collision at the current velocity and desired braking deceleration.
   - For many steps along the route up to that distance, determine whether the vehicle would be likely to hit (more precisely, get sufficiently close) to the pedestrian.  Use a 1m lateral distance and 3m longitudinal distance buffer in your vehicle geometry. 
   - For the distance-to-collision you determine above, use the longitudinal planner to determine your desired acceleration.

   Determine how quickly you can run your planner.  If it is too slow (<5Hz), try using the vehicle-pedestrian distance to dynamically determine your collision checking resolution.

2. Modify the `PedestrianYielder` so that it selects a subset of pedestrians for which the above calculations should be performed.
3. Adapt the motion planner to slow gently for distant crossing pedestrians to boost rate of progress after the pedestrian crosses the vehicle's route.  Use the Perception group's estimated velocity for each pedestrian.  
4. Using the `--variant real_real` will run on the real vehicle using trajectory predictions from the Perception group.
5. Integrate as necessary.


## Infra Group
-	Have at least one team member complete safety lookout & driver training.

Part 1:
1. Develop a systematic method for saving camera data for the Calibration and Vision groups.  Work closely with them to figure out a method to test their methods from logged data.
2. Write documentation for this method, detailing usage and performance issues.
3. Use Git to create a pull request to `s2025_teamX`, and have your PR approved.

Part 2: 
1. Develop a visualizer for rapidly plotting detected pedestrian trajectories from log data (e.g., matplotlib).
2. Work with the Control & Dynamics team to test the integrated system.
3. Develop metrics for pedestrian safety and passenger comfort. Analyze logs from tests.
4. Integrate as necessary.

