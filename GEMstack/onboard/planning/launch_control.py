import rospy

class LaunchControl:
    def __init__(self, stage_duration, stop_threshold):
        self.enable_launch_control = True
        self.stage_duration = stage_duration
        self.stop_threshold = stop_threshold
        self._launch_start_time = None

    def reset(self):
        self.enable_launch_control = True
        self._launch_start_time = None


    def apply_launch_control(self, cmd, vehicle_velocity):

        if vehicle_velocity < self.stop_threshold:
            self.reset()
            
        if self.enable_launch_control:
            print("launch control active")
            if self._launch_start_time is None:
                self._launch_start_time = rospy.get_time()

            elapsed = rospy.get_time() - self._launch_start_time

            if elapsed < self.stage_duration:
                cmd.accelerator_pedal_position = 0.0
                cmd.brake_pedal_position = 1.0
            elif elapsed < 2 * self.stage_duration:
                cmd.accelerator_pedal_position = 1.0
                cmd.brake_pedal_position = 1.0
            elif elapsed < 3 * self.stage_duration:
                cmd.accelerator_pedal_position = 1.0
                cmd.brake_pedal_position = 0.0
            else:
                self.enable_launch_control = False

        return cmd