import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np

def create_comparison_plot(t_base, data_base, t_tune, data_tune, xlabel, ylabel, title, legend, save_path=None):
    plt.figure()
    plt.plot(t_base, data_base, label=legend[0])
    plt.plot(t_tune, data_tune, label=legend[1])
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.grid(True)
    if save_path:
        plt.savefig(save_path, dpi=600)

def main():
    log_folder_base = '2025-02-09_15-47-37 (before tuning)'
    log_folder_tune = '2025-02-13_21-10-39 (fine tuning)'

    df_base = pd.read_csv(log_folder_base + '/PurePursuitTrajectoryTracker_debug.csv')
    df_tune = pd.read_csv(log_folder_tune + '/PurePursuitTrajectoryTracker_debug.csv')

    save_figures =True

    plots_folder = os.path.join(log_folder_base + '--' + log_folder_tune, 'plots')
    os.makedirs(plots_folder, exist_ok=True)

    t_base = df_base['curr pt[0] vehicle time'].tolist()
    x_base = df_base['curr pt[0]'].tolist()
    y_base = df_base['curr pt[1]'].tolist()
    v_base = df_base['current speed (m/s)'].tolist()
    yaw_base = df_base['current yaw (rad)'].tolist()
    xd_base = df_base['desired pt[0]'].tolist()
    yd_base = df_base['desired pt[1]'].tolist()
    vd_base = df_base['desired speed (m/s)'].tolist()
    yawd_base = df_base['desired yaw (rad)'].tolist()
    cte_base = df_base['crosstrack error'].tolist()
    front_wheel_angle_base = df_base['front wheel angle (rad)'].tolist()
    accel_base = df_base['output accel (m/s^2)'].tolist()

    t_tune = df_tune['curr pt[0] vehicle time'].tolist()
    x_tune = df_tune['curr pt[0]'].tolist()
    y_tune = df_tune['curr pt[1]'].tolist()
    v_tune = df_tune['current speed (m/s)'].tolist()
    yaw_tune = df_tune['current yaw (rad)'].tolist()
    xd_tune = df_tune['desired pt[0]'].tolist()
    yd_tune = df_tune['desired pt[1]'].tolist()
    vd_tune = df_tune['desired speed (m/s)'].tolist()
    yawd_tune = df_tune['desired yaw (rad)'].tolist()
    cte_tune = df_tune['crosstrack error'].tolist()
    front_wheel_angle_tune = df_tune['front wheel angle (rad)'].tolist()
    accel_tune = df_tune['output accel (m/s^2)'].tolist()

    create_comparison_plot(t_base, np.array(x_base) - np.array(xd_base), t_tune, np.array(x_tune) - np.array(xd_tune), '$t$ (s)', 'Error in $x(t)$ (m)', 'Error in x Comparison', ['Baseline', 'Tuned'], os.path.join(plots_folder, 'error_x_comparison.png') if save_figures else None)
    create_comparison_plot(t_base, np.array(y_base) - np.array(yd_base), t_tune, np.array(y_tune) - np.array(yd_tune), '$t$ (s)', 'Error in $y(t)$ (m)', 'Error in y Comparison', ['Baseline', 'Tuned'], os.path.join(plots_folder, 'error_y_comparison.png') if save_figures else None)
    create_comparison_plot(t_base, np.array(v_base) - np.array(vd_base), t_tune, np.array(v_tune) - np.array(vd_tune), '$t$ (s)', 'Error in $v(t)$ (m/s)', 'Error in Speed Comparison', ['Baseline', 'Tuned'], os.path.join(plots_folder, 'error_v_comparison.png') if save_figures else None)
    create_comparison_plot(t_base, np.array(yaw_base) - np.array(yawd_base), t_tune, np.array(yaw_tune) - np.array(yawd_tune), '$t$ (s)', 'Error in $yaw(t)$ (rad)', 'Error in Yaw Comparison', ['Baseline', 'Tuned'], os.path.join(plots_folder, 'error_yaw_comparison.png') if save_figures else None)
    create_comparison_plot(t_base, cte_base, t_tune, cte_tune, '$t$ (s)', 'Crosstrack Error (m)', 'Crosstrack Error Comparison', ['Baseline', 'Tuned'], os.path.join(plots_folder, 'cte_comparison.png') if save_figures else None)
    create_comparison_plot(t_base, front_wheel_angle_base, t_tune, front_wheel_angle_tune, '$t$ (s)', 'Front Wheel Angle (rad)', 'Front Wheel Angle Comparison', ['Baseline', 'Tuned'], os.path.join(plots_folder, 'front_wheel_angle_comparison.png') if save_figures else None)
    create_comparison_plot(t_base, accel_base, t_tune, accel_tune, '$t$ (s)', 'Acceleration (m/s²)', 'Acceleration Comparison', ['Baseline', 'Tuned'], os.path.join(plots_folder, 'accel_comparison.png') if save_figures else None)

    plt.show()


if __name__ == '__main__':
    main()
