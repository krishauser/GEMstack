import re
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
"""
To generate the calibration text file with lists of measured acceleration, output acceleration, and speed, follow these steps:

    1. **Initialization**: 
    Inside the '__init__' method of the 'PurePursuitTrajectoryTracker', 
    initialize three lists to store the measured acceleration, the output acceleration, and the speed values.

        def __init__(self):
            self.acc_measured = []
            self.acc_output = []
            self.speed = []

    2. **Data Collection**: 
    Within the 'update' function of the controller, append the current vehicle state values to these lists. 

        def update(self, vehicle: VehicleState, trajectory: Trajectory):
            self.acc_measured.append(vehicle.acceleration)
            self.acc_output.append(accel) 
            self.speed.append(vehicle.v)
            print('acc_measured_list:', self.acc_measured)
            print('acc_out_list:', self.acc_output)
            print('speed_list:', self.speed)

    3. **Generate the Text File**: 
    Run your real-world test by executing the appropriate command. 
    Ensure the tested route: GEMstack/knowledge/routes/forward_15m.csv is specified correctly in your 'fixed_route.yaml'. 
    This will capture the printed lists into the 'acc_cali.txt' file.

        python3 main.py launch/fixed_route.yaml > acc_cali.txt
"""

def read_last_list_from_file(filename, keyword):
    """Extracts the last occurrence of a list in the file after a specific keyword."""
    with open(filename, 'r') as file:
        content = file.read()

    # Finding the last occurrence of the keyword
    last_index = content.rfind(keyword)
    if last_index == -1:
        return None  # Keyword not found in the file

    # Extract the part of the file after this index
    data_content = content[last_index + len(keyword):].strip()
    match = re.search(r'\[.*?\](?=\s|$)', data_content, re.DOTALL)
    if match:
        return eval(match.group(0))
    return None

def extract_acc_cali_data(filename):
    """ Extracts acceleration and speed data from a calibration file. """
    acc_measured = read_last_list_from_file(filename, 'acc_measured_list:')
    acc_out = read_last_list_from_file(filename, 'acc_out_list:')
    speed = read_last_list_from_file(filename, 'speed_list:')

    return acc_measured, acc_out, speed

def friction_model(speed, a, b, c):
    """Model for friction dependent on speed."""
    return a * speed**2 + b * speed + c

def perform_curve_fitting(speed, acc_msd, acc_out):
    """Performs curve fitting to find the friction model coefficients."""
    friction_force = np.array(acc_out) - np.array(acc_msd)
    popt, pcov = curve_fit(friction_model, speed, friction_force)
    return popt

def plot_friction_model(speed, acc_msd, acc_out, coefficients):
    """Plots the measured friction force against the speed and the fitted model."""
    plt.scatter(speed, np.array(acc_out) - np.array(acc_msd), label='Data (Friction Force)')
    plt.plot(speed, friction_model(np.array(speed), *coefficients), color='red', label='Fitted Model')
    plt.xlabel('Speed (m/s)')
    plt.ylabel('Friction Force (m/s²)')
    plt.title('Speed-Dependent Friction Modeling')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    filename = "acc_cali.txt"
    acc_measured, acc_out, speed = extract_acc_cali_data(filename)

    if acc_measured and acc_out and speed:
        coefficients = perform_curve_fitting(np.array(speed), acc_measured, acc_out)
        print("Fitted coefficients:", coefficients)
        plot_friction_model(speed, acc_measured, acc_out, coefficients)
    else:
        print("Data extraction failed or incomplete.")