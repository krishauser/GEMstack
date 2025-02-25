import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np

# Most of the code in this file is copied from ./tuning_logs/log_plot.py by Mikayel Aramyan (mikayel2)

def create_plot(t, actual, desired, xlabel, ylabel, title, legend, save_path=None):
    plt.figure()
    plt.plot(t, actual)
    plt.plot(t, desired)
    overshoot = np.max(np.array(actual) - np.array(desired))
    rmse = np.sqrt(np.mean((np.array(actual) - np.array(desired))**2))
    print(f'{title} - Maximum Overshoot: {overshoot}, RMSE: {rmse}')
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(f'{title} (Max Overshoot: {overshoot:.2f}, RMSE: {rmse:.2f})')
    plt.legend(legend)
    plt.grid(True)
    if save_path:
        plt.savefig(save_path, dpi=600)
        plt.close()

def create_error_plot(t, error, xlabel, ylabel, title, save_path=None):
    plt.figure()
    plt.plot(t, error)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    if save_path:
        plt.savefig(save_path, dpi=600)
        plt.close()

def main(log_folder):
    if log_folder is None:
        return
    df = pd.read_csv(log_folder + '/PurePursuitTrajectoryTracker_debug.csv')
    save_figures =True

    plots_folder = os.path.join(log_folder, 'plots')
    os.makedirs(plots_folder, exist_ok=True)

    t = df['curr pt[0] vehicle time'].tolist()
    x = df['curr pt[0]'].tolist()
    y = df['curr pt[1]'].tolist()
    v = df['current speed (m/s)'].tolist()
    yaw = df['current yaw (rad)'].tolist()

    xd = df['desired pt[0]'].tolist()
    yd = df['desired pt[1]'].tolist()
    vd = df['desired speed (m/s)'].tolist()
    yawd = df['desired yaw (rad)'].tolist()

    cte = df['crosstrack error'].tolist()
    front_wheel_angle = df['front wheel angle (rad)'].tolist()
    accel = df['output accel (m/s^2)'].tolist()



    rmse_cte = np.sqrt(np.mean(np.array(cte)**2))
    print(f'RMSE (cte): {rmse_cte}')

    max_accel_error = np.max((np.array(accel) - 1.0)**2)
    print(f'Maximum (acceleration - 1.0)^2: {max_accel_error}')

    rms_forward_acceleration = np.sqrt(np.mean(np.array(accel)**2))
    print(f'RMS of Acceleration: {rms_forward_acceleration}')

    dt = np.mean(np.diff(t))
    jerk = np.gradient(np.array(accel), dt)
    rms_jerk = np.sqrt(np.mean(jerk**2))
    print(f'RMS of Jerk: {rms_jerk}')

    speed_error = np.array(v) - np.array(vd)
    rms_speed_error = np.sqrt(np.mean(speed_error**2))
    print(f'RMS of Speed Error: {rms_speed_error}')

    w1, w2, w3 = 0.5, 0.3, 0.2
    comfort_index = w1 * rms_forward_acceleration + w2 * rms_jerk + w3 * rms_speed_error
    print(f'Comfort Index: {comfort_index}')


    create_plot(t, y, yd, '$t$ (s)', '$y(t)$, $y_{d}(t)$ (m)', 'Actual and Desired y', ['Actual $y(t)$', 'Desired $y_{d}(t)$'], os.path.join(plots_folder, 'y_vs_yd.png') if save_figures else None)
    create_error_plot(t, np.array(y) - np.array(yd), '$t$ (s)', 'Error in $y(t)$ (m)', 'Error between Actual and Desired $y(t)$', os.path.join(plots_folder, 'error_y.png') if save_figures else None)

    create_plot(t, x, xd, '$t$ (s)', '$x(t)$, $x_{d}(t)$ (m)', 'Actual and Desired x', ['Actual $x(t)$', 'Desired $x_{d}(t)$'], os.path.join(plots_folder, 'x_vs_xd.png') if save_figures else None)
    create_error_plot(t, np.array(x) - np.array(xd), '$t$ (s)', 'Error in $x(t)$ (m)', 'Error between Actual and Desired $x(t)$', os.path.join(plots_folder, 'error_x.png') if save_figures else None)

    create_plot(t, v, vd, '$t$ (s)', '$v(t)$, $v_{d}(t)$ (m/s)', 'Actual and Desired v', ['Actual $v(t)$', 'Desired $v_{d}(t)$'], os.path.join(plots_folder, 'v_vs_vd.png') if save_figures else None)
    create_error_plot(t, np.array(v) - np.array(vd), '$t$ (s)', 'Error in $v(t)$ (m/s)', 'Error between Actual and Desired $v(t)$', os.path.join(plots_folder, 'error_v.png') if save_figures else None)

    plt.figure()
    plt.plot(t, cte)
    rmse_cte = np.sqrt(np.mean(np.array(cte)**2))
    print(f'RMSE (cte): {rmse_cte}')
    plt.xlabel('$t$ (s)')
    plt.ylabel('Crosstrack Error (m)')
    plt.title(f'Crosstrack Error over Time (RMSE: {rmse_cte:.2f})')
    plt.grid(True)
    if save_figures:
        plt.savefig(os.path.join(plots_folder, 'cte.png'), dpi=600)
        plt.close()

    plt.figure()
    plt.plot(t, front_wheel_angle)
    plt.xlabel('$t$ (s)')
    plt.ylabel('Front Wheel Angle (rad)')
    plt.title('Front Wheel Angle over Time')
    plt.grid(True)
    if save_figures:
        plt.savefig(os.path.join(plots_folder, 'front_wheel_angle.png'), dpi=600)
        plt.close()

    plt.figure()
    plt.plot(t, accel)
    plt.xlabel('$t$ (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.title('Acceleration over Time')
    plt.grid(True)
    if save_figures:
        plt.savefig(os.path.join(plots_folder, 'accel.png'), dpi=600)
        plt.close()




if __name__ == '__main__':
    # Change this to the log folder you want to plot if running this script manually
    log_folder = '2025-02-13_21-10-39'
    main(log_folder)
