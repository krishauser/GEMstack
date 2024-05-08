import matplotlib.pyplot as plt

class ControllerEvaluator:
    """A performance evaluator for evaluating the controllers."""
    def __init__(self, ref_path, actual_path, time_stamps):
        # Check if both paths and time_stamps have the same length
        if not (len(ref_path) == len(actual_path) == len(time_stamps)):
            raise ValueError("Reference path, actual path, and time stamps must be of the same length.")
        self.ref_path = ref_path
        self.actual_path = actual_path
        self.time_stamps = time_stamps

    def plot_parameter_over_time(self, parameter_index):
        # Extract the parameter values from the paths
        ref_values = [state[parameter_index] for state in self.ref_path]
        actual_values = [state[parameter_index] for state in self.actual_path]

        # Create the plot
        plt.figure(figsize=(10, 6))
        plt.plot(self.time_stamps, ref_values, label='Reference', marker='o')
        plt.plot(self.time_stamps, actual_values, label='Actual', linestyle='--', marker='x')
        
        # Set the title and labels depending on the parameter index
        titles = ['X Position', 'Y Position', 'Orientation']
        ylabel = titles[parameter_index] if parameter_index in [0, 1, 2] else 'Parameter Value'
        
        plt.title(f'{titles[parameter_index]} Over Time' if parameter_index in [0, 1, 2] else 'Parameter Over Time')
        plt.xlabel('Time')
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_paths(self):
        # Unzip the paths into separate lists for plotting
        ref_x, ref_y, _ = zip(*self.ref_path)
        actual_x, actual_y, _ = zip(*self.actual_path)

        plt.figure(figsize=(10, 6))
        plt.plot(ref_x, ref_y, 'b', label='Reference Path', marker='o')
        plt.plot(actual_x, actual_y, 'r', label='Actual Path', marker='x')
        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.title('Reference vs Actual Path')
        plt.legend()
        plt.axis('equal')  # This ensures that the scale on both axes is the same
        plt.grid(True)
        plt.show()

    def calculate_position_error(self):
        # Calculate the MSE of the position error
        position_errors = [(ref[0] - act[0])**2 + (ref[1] - act[1])**2 
                           for ref, act in zip(self.ref_path, self.actual_path)]
        mse_position = sum(position_errors) / len(self.ref_path)
        return mse_position

    def calculate_orientation_error(self):
        # Calculate the MSE of the orientation error
        orientation_errors = [(ref[2] - act[2])**2 
                              for ref, act in zip(self.ref_path, self.actual_path)]
        mse_orientation = sum(orientation_errors) / len(self.ref_path)
        return mse_orientation

if __name__ == "__main__":
    
    # Example inputs
    ref_path = [[1, 0, 0], [2, 0, 0], [3, 0, 0], [4, 0, 0], [5, 0, 0]]
    actual_path = [[1.1, 0, 0], [1.9, 0, 0], [3.1, 0, 0], [3.9, 0, 0], [5.1, 0, 0]]

    rate = 10.0
    time_stamps = [1 / rate * i for i in range(len(ref_path))]

    # Assume ref_path, actual_path, and time_stamps are already defined
    evaluator = ControllerEvaluator(ref_path, actual_path, time_stamps)

    # Plot the actual and reference paths
    evaluator.plot_paths()

    # Plot X Position Over Time
    evaluator.plot_parameter_over_time(0)

    # Plot Y Position Over Time
    evaluator.plot_parameter_over_time(2)

    # Calculate position and orientation error
    e_position = evaluator.calculate_position_error()
    e_orientation = evaluator.calculate_orientation_error()


