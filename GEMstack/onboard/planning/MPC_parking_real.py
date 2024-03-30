import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle


#Vehicle Param
L = 2.0         
delta_max = np.pi/4 # max steering angle  
dt = 0.2
def simulate_vehicle(x, y, theta, v, delta, a, dt, L):
    """
    Simulate vehicle dynamics for one time step.
    """
    x_next = x + v * np.cos(theta) * dt
    y_next = y + v * np.sin(theta) * dt
    theta_next = theta + v * np.tan(delta) / L * dt
    v_next = v + a * dt
    return x_next, y_next, theta_next, v_next

# Setup the MPC problem 
def setup_mpc(setup_type,N, dt, L, x0, y0, theta0, v0, x_goal, y_goal, theta_goal, v_goal, obstacles):
    """
    Setup and solve the MPC problem.
    Returns the first control inputs (v, delta) from the optimized trajectory.
    """

    # Steering angle and velocity limits
    delta_min, delta_max = -np.pi/4, np.pi/4

    if setup_type == 'front':
        a_min, a_max = -2, 2
        v_min, v_max = 0, 1
    elif setup_type == 'back':
        a_min, a_max = -0.5, 0.5
        v_min, v_max = -1, 0.01


    opti = ca.Opti()  # Create an optimization problem

    # State
    X = opti.variable(4, N+1)  # [x, y, theta, v]
    U = opti.variable(2, N)    # [a, delta]

    # Initial constraints
    opti.subject_to(X[:,0] == [x0, y0, theta0, v0])

    # Dynamics constraints
    for k in range(N):
        x_next = X[0,k] + X[3,k]*ca.cos(X[2,k])*dt
        y_next = X[1,k] + X[3,k]*ca.sin(X[2,k])*dt
        theta_next = X[2,k] + X[3,k]/L*ca.tan(U[1,k])*dt
        v_next = X[3,k] + U[0,k]*dt
        
        opti.subject_to(X[0,k+1] == x_next)
        opti.subject_to(X[1,k+1] == y_next)
        opti.subject_to(X[2,k+1] == theta_next)
        opti.subject_to(X[3,k+1] == v_next)

        # Obstacle constraints
        if setup_type == 'back':
            for obs in obstacles:
                obs_x, obs_y, obs_w, obs_h = obs
                distance_squared = (X[0,k] - obs_x)**2 + (X[1,k] - obs_y)**2
                opti.subject_to(distance_squared >= 0.1)


    # Control costraints
    opti.subject_to(opti.bounded(a_min, U[0,:], a_max))
    opti.subject_to(opti.bounded(delta_min, U[1,:], delta_max))
    opti.subject_to(opti.bounded(v_min, X[3,:], v_max))

    # objective
    opti.minimize(ca.sumsqr(X[0:4,-1] - [x_goal, y_goal, theta_goal, v_goal]))

    # Solver
    opts = {"ipopt.print_level": 0, "print_time": 0}
    opti.solver("ipopt", opts)

    sol = opti.solve()
    x_sol = sol.value(X[0,:])
    y_sol = sol.value(X[1,:])
    theta_sol = sol.value(X[2,:])
    v_sol = sol.value(X[3,:])[0]
    acc_sol = sol.value(U[0,:])[0]
    delta_sol = sol.value(U[1,:])[0]

    return v_sol, delta_sol, acc_sol


# Initial vehicle state
x_current, y_current, theta_current, v_current = 0, 6, 0, 0

# Goal state
x_goal, y_goal, theta_goal, v_goal = 5, 11.5, -np.pi/2, 0

# Back waypoint
x_back, y_back, theta_back, v_back = 5, y_goal - 5.5, -np.pi/2, 0

# Simulation parameters
N = 10
dt = 0.2
L = 2.0
 
setup_type1 = 'front'
setup_type2 = 'back'
w_vehicle = 1
h_vehicle = 3

# List of obstacles (as before)
obstacles = [(7,1.5,1,3),(1.5,1.5,1,3),(9,1.5,1,3),(1.5,11.5,1,3),(5,1.5,1,3),(7,11.5,1,3),(9,11.5,1,3)]

# Initialize lists to store the vehicle's trajectory for plotting
x_trajectory = [x_current]
y_trajectory = [y_current]

phase_flag = 1

# From intial position to back waypoint
while True:
    if phase_flag == 1:
        # Solve the MPC problem with the current state as the starting point
        v_opt, delta_opt, a_opt = setup_mpc(setup_type1,N, dt, L, x_current, y_current, theta_current, v_current, x_back, y_back, theta_back, v_back, obstacles)
        print("delta1: ", delta_opt)
        if v_opt == None or delta_opt == None:
            print("Solution doesn't exist.")
            break

        # Simulate vehicle dynamics for one time step using the optimized control inputs
        x_next, y_next, theta_next, v_next = simulate_vehicle(x_current, y_current, theta_current, v_opt, delta_opt, a_opt, dt, L)
        
        # Update current state for the next iteration
        x_current, y_current, theta_current, v_current = x_next, y_next, theta_next, v_next

        # Store the new position
        x_trajectory.append(x_current)
        y_trajectory.append(y_current)

        # Termination condition if the vehicle reaches the goal area
        # if np.sqrt((x_current - x_back)**2 + (y_current - y_back)**2) <= 0.1:  # threshold for reaching the goal
        #     print("Back point reached.")
        #     phase_flag = 2
        if np.sqrt((x_current - x_goal)**2 + (y_current - y_goal)**2) <= 0.02 or v_current <= 0.001:  # threshold for reaching the goal
            print("Back point reached.")
            phase_flag = 2
            
        
    elif phase_flag == 2:
        # Solve the MPC problem with the current state as the starting point
        v_opt, delta_opt,a_opt = setup_mpc(setup_type2,N, dt, L, x_current, y_current, theta_current, v_current, x_goal, y_goal, theta_goal, v_goal, obstacles)
        print("delta2: ", delta_opt)
        if v_opt == None or delta_opt == None:
            print("Solution doesn't exist.")
            break

        # Simulate vehicle dynamics for one time step using the optimized control inputs
        x_next, y_next, theta_next, v_next = simulate_vehicle(x_current, y_current, theta_current, v_opt, delta_opt, a_opt, dt, L)
        
        # Update current state for the next iteration
        x_current, y_current, theta_current, v_current = x_next, y_next, theta_next, v_next

        # Store the new position
        x_trajectory.append(x_current)
        y_trajectory.append(y_current)

        # Termination condition if the vehicle reaches the goal area
        # if np.sqrt((x_current - x_goal)**2 + (y_current - y_goal)**2) <= 0.2:  # threshold for reaching the goal
        #     print("Goal reached.")
        #     break
        if  np.sqrt((x_current - x_goal)**2 + (y_current - y_goal)**2) <= 0.1 and v_current > -0.001:  # threshold for reaching the goal
            print("Goal reached.")
            break


# Creating the figure and axis for the animation
vehicle_size = [1,3]
fig, ax = plt.subplots()
ax.set_xlim(0, 10)
ax.set_ylim(0, 15)

# Plotting static elements (e.g., obstacles)
for ox, oy, ow, oh in obstacles:
    obstacle_rect = Rectangle((ox - ow/2, oy - oh/2), ow, oh, color="gray", alpha=0.5)
    ax.add_patch(obstacle_rect)

# Initial vehicle representation (this will be updated during the animation)
vehicle_rect = Rectangle((0 - vehicle_size[0]/2, 0 - vehicle_size[1]/2), vehicle_size[0], vehicle_size[1], fill=False, color="blue")
ax.add_patch(vehicle_rect)

# Line object to show the trajectory
trajectory_line, = ax.plot([], [], 'b-', lw=2)

# Initialization function for the animation
def init():
    trajectory_line.set_data([], [])
    return trajectory_line, vehicle_rect,

# Animation update function
def update(frame):
    # Updating the vehicle's position
    x, y = x_trajectory[frame], y_trajectory[frame]
    vehicle_rect.set_xy((x - vehicle_size[0]/2, y - vehicle_size[1]/2))
    
    # Updating the trajectory line
    trajectory_line.set_data(x_trajectory[:frame+1], y_trajectory[:frame+1])
    return trajectory_line, vehicle_rect,

# Creating the animation
ani = FuncAnimation(fig, update, frames=range(len(x_trajectory)), init_func=init, blit=True, interval=100)

plt.show()


