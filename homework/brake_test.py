#!/usr/bin/env python

import rospy
import time
import numpy as np
import csv
from dataclasses import dataclass
from typing import List

# Import your specific modules (adjust the import paths as needed)
from your_package.utils import settings
from your_package.state import VehicleState, ObjectPose, ObjectFrameEnum
from your_package.knowledge.vehicle.dynamics import pedal_positions_to_acceleration
from your_package.hardware.vehicle.gem import GEMInterface, GEMVehicleCommand, GEMVehicleReading

class BrakeTestRecorder:
    def __init__(self):
        rospy.init_node('brake_test_recorder', anonymous=True)
        
        # Initialize the GEM interface
        self.gem = GEMInterface()
        self.gem.start()
        
        # Data storage
        self.times = []
        self.speeds = []
        self.accelerations = []
        self.brake_positions = []
        
        # Test parameters
        self.test_interval = 0.1  # seconds
        self.target_speed = 8.33  # m/s (30 kph)
        self.start_time = None
        
    def run_test(self):
        # Wait for the vehicle to reach target speed
        reading = self.gem.get_reading()
        rospy.loginfo(f"Current speed: {reading.speed} m/s, waiting for {self.target_speed} m/s")
        
        while reading.speed < self.target_speed - 0.5:  # Allow 0.5 m/s tolerance
            rospy.loginfo(f"Current speed: {reading.speed} m/s, waiting for {self.target_speed} m/s")
            time.sleep(1.0)
            reading = self.gem.get_reading()
        
        rospy.loginfo(f"Starting brake test from {reading.speed} m/s")
        self.start_time = rospy.get_time()
        
        rate = rospy.Rate(1/self.test_interval)  # 10 Hz for 0.1s intervals
        
        # Initialize with current state but zero accelerator and full brake
        brake_position = 0.0
        
        # Continue until vehicle is nearly stopped
        while reading.speed > 0.5:  # Consider stopped below 0.5 m/s
            current_time = rospy.get_time() - self.start_time
            
            # Increase brake gradually 
            brake_position += 0.1
            if brake_position > 1.0:
                brake_position = 1.0
                
            # Create and send command with full brake
            cmd = self.gem.command_from_reading(reading)
            cmd.accelerator_pedal_position = 0.0
            cmd.brake_pedal_position = brake_position
            self.gem.send_command(cmd)
            
            # Record data
            state = reading.to_state()
            self.times.append(current_time)
            self.speeds.append(reading.speed)
            self.accelerations.append(state.acceleration)
            self.brake_positions.append(brake_position)
            
            # Log current status
            rospy.loginfo(f"Time: {current_time:.2f}s, Speed: {reading.speed:.2f} m/s, "
                         f"Acceleration: {state.acceleration:.2f} m/s², Brake: {brake_position:.2f}")
            
            # Wait for next interval
            rate.sleep()
            
            # Get new reading
            reading = self.gem.get_reading()
        
        # Test complete
        rospy.loginfo("Brake test complete, saving data...")
        self.save_data()
        
    def save_data(self):
        # Save data to CSV file
        filename = f"brake_test_data_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Speed (m/s)', 'Acceleration (m/s²)', 'Brake Position'])
            for i in range(len(self.times)):
                writer.writerow([self.times[i], self.speeds[i], self.accelerations[i], self.brake_positions[i]])
        
        rospy.loginfo(f"Data saved to {filename}")
        
    def cleanup(self):
        # Stop the GEM interface
        self.gem.stop()
        

if __name__ == '__main__':
    try:
        recorder = BrakeTestRecorder()
        recorder.run_test()
        recorder.cleanup()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in brake test: {e}")