import matplotlib.pyplot as plt
# import numpy as np
import re

class Evaluator:
    """A performance evaluator for evaluating the controllers."""
    def __init__(self, ref_path, actual_path):
        # Check if both paths have the same length
        if len(ref_path) != len(actual_path):
            raise ValueError("Reference path and actual path must be of the same length.")
        self.ref_path = ref_path
        self.actual_path = actual_path

    def plot(self):
        # Unzip the paths into separate lists for plotting
        ref_x, ref_y, _ = zip(*self.ref_path)
        actual_x, actual_y, _ = zip(*self.actual_path)

        position_errors, mse_position = self.calculate_position_error()
        orientation_errors, mse_orientation = self.calculate_orientation_error()
        t = range(len(position_errors))
        # Create the plot
        plt.figure(figsize=(10, 6))
        plt.plot(ref_x, ref_y, label='Reference Path', marker='o')
        plt.plot(actual_x, actual_y, label='Actual Path', marker='x')
        plt.title('Reference vs Actual Path')
        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        plt.show()

        # Create the plot
        plt.figure(figsize=(10, 6))
        plt.plot(t, position_errors, label='position_errors', marker='o')
        # plt.plot(actual_x, actual_y, label='Actual Path', marker='x')
        plt.title('position_errors')
        plt.xlabel('time')
        plt.ylabel('position_errors')
        plt.legend()
        # plt.axis('equal')
        plt.grid(True)
        plt.show()

        # Create the plot
        plt.figure(figsize=(10, 6))
        plt.plot(t, orientation_errors, label='orientation_errors', marker='o')
        # plt.plot(actual_x, actual_y, label='Actual Path', marker='x')
        plt.title('orientation_errors')
        plt.xlabel('time')
        plt.ylabel('orientation_errors')
        plt.legend()
        # plt.axis('equal')
        plt.grid(True)
        plt.show()



    def calculate_position_error(self):
        # Calculate the MSE of the position error
        position_errors = [(ref[0] - act[0])**2 + (ref[1] - act[1])**2 
                           for ref, act in zip(self.ref_path, self.actual_path)]
        
        position_errors_100 = [((abs(ref[0] - act[0]) / ref[0] if ref[0] != 0 else 0) + 
                    (abs(ref[1] - act[1]) / ref[1] if ref[1] != 0 else 0)) * 100 / 2
                   for ref, act in zip(self.ref_path, self.actual_path)]

        mse_position = sum(position_errors) / len(self.ref_path)
        return position_errors, mse_position

    def calculate_orientation_error(self):
        # Calculate the MSE of the orientation error
        orientation_errors = [(ref[2] - act[2])**2 
                              for ref, act in zip(self.ref_path, self.actual_path)]
        
        # orientation_errors_100 = [((abs(ref[0] - act[0]) / ref[0] if ref[0] != 0 else 0) + 
        #             (abs(ref[1] - act[1]) / ref[1] if ref[1] != 0 else 0)) * 100 / 2
        #            for ref, act in zip(self.ref_path, self.actual_path)]

        mse_orientation = sum(orientation_errors) / len(self.ref_path)
        return orientation_errors, mse_orientation
    
def extract_last_paths(filename):
    with open(filename, 'r') as file:
        content = file.read()
    
    # Finding the last occurrence of 'REF PATH:' and 'ACTUAL PATH:'
    last_ref_index = content.rfind('REF PATH:')
    last_actual_index = content.rfind('ACTUAL PATH:')
    
    if last_ref_index == -1 or last_actual_index == -1:
        return None, None  # These strings were not found in the file

    # Extract the parts of the file after these indices
    ref_path_content = content[last_ref_index + len('REF PATH:'):].strip()
    actual_path_content = content[last_actual_index + len('ACTUAL PATH:'):].strip()
    
    # Assuming each list ends with a newline and starts on a new line
    # ref_path_list = ref_path_content.split('\n', 1)[0].strip().split()
    # actual_path_list = actual_path_content.split('\n', 1)[0].strip().split()





    # Regular expression to find the REF PATH data
    match = re.search(r"(\[\[.*?\]\])\s+ACTUAL PATH:", ref_path_content, re.DOTALL)
    if match:
        ref_path_data = match.group(1)
        # Convert the string to a Python list
        ref_path_list = eval(ref_path_data)
        # print("ref paths found:",ref_path_list)
    else:
        print("REF PATH data not found")

    match = re.search(r"(\[\[.*?\]\])", actual_path_content, re.DOTALL)

    if match:
        # Extracts the captured group which contains the paths
        actual_path_data = match.group(1)
        
        actual_path_list = eval(actual_path_data)
        # print("Actual paths found:", actual_path_list)

    else:
        print("No match found.")

        
       
    return ref_path_list, actual_path_list

# Usage
filename = 'test2.txt'
ref_paths, actual_paths = extract_last_paths(filename)
print("REF PATHS:", ref_paths)
print("ACTUAL PATHS:", actual_paths)

# if __name__ == "__main__":


eva = Evaluator(ref_paths, actual_paths)
eva.plot()
    