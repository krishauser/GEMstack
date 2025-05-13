import rospy

class LaunchControl:
    def __init__(self, stage_duration, stop_threshold):
        self.enable_launch_control = True
        self.stage_duration = stage_duration
        self.stop_threshold = stop_threshold
        self._launch_start_time = None

    def reset(self):
        self.enable_launch_control = True
        self._launch_start_time = 0


    def apply_launch_control(self, cmd, vehicle_velocity):

        if self._launch_start_time == None:
            self._launch_start_time = rospy.get_time()
        elapsed = rospy.get_time() - self._launch_start_time
        if self.enable_launch_control:
            print("launch control active")

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

        if vehicle_velocity < self.stop_threshold and elapsed > 3 * self.stage_duration:
            # self.reset()
            self.enable_launch_control = False

        return cmd