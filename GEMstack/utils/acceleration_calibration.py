import re
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

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
