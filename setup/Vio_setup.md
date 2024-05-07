To use vio instead of GPS for localization:

    -install rtabmap on your system "sudo apt install ros--rtabmap-ros" (I already installed on gem04 vehicle)

    -create your yaml launch file and for the field "drive.perception.state_estimation : VIOSlamEstimator " use VIOSlamEstimator component

    -run command python3 main.py launch/<your_file>.yaml.You can configure the rgbdrtabmap.launch in 'launch/rgbdrtabmap.launch' file as needed to visualize the map creation in real time. Odometry information is published and the VIOSlamEstimator component updates the vehicle's VehicleState object where you can extract the ego pose.