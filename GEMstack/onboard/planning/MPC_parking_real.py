import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle


#Vehicle Param
L = 2.0         
delta_max = np.pi/4 # max steering angle  
safety_margin = 0.1  
dt = 0.1   

def simulate_vehicle(x, y, theta, v, delta, dt, L):
    """
    Simulate vehicle dynamics for one time step.
    """
    x_next = x + v * np.cos(theta) * dt
    y_next = y + v * np.sin(theta) * dt
    theta_next = theta + v * np.tan(delta) / L * dt
    return x_next, y_next, theta_next

# Setup the MPC problem 
def setup_mpc(setup_type,N, dt, L, x_start, y_start, theta_start, x_goal, y_goal, theta_goal, obstacles):
    """
    Setup and solve the MPC problem.
    Returns the first control inputs (v, delta) from the optimized trajectory.
    """

    # Steering angle and velocity limits
    delta_min, delta_max = -np.pi/4, np.pi/4

    if setup_type == 'front':
        a_min, a_max = 0, 0.5
        v_min, v_max = 0, 1
    elif setup_type == 'back':
        a_min, a_max = -0.5, 0
        v_min, v_max = -1, 0


    opti = ca.Opti()  # Create an optimization problem

    # Variables
    X = opti.variable(N+1)
    Y = opti.variable(N+1)
    Theta = opti.variable(N+1)
    V = opti.variable(N+1)
    Delta = opti.variable(N)
    A = opti.variable(N)

    # Objective function (for simplicity, just trying to reach the goal)
    objective = ca.sum1((X - x_goal)**2 + (Y - y_goal)**2 + (Theta - theta_goal)**2)

    # Dynamics constraints
    constraints = [X[0] == x_start, Y[0] == y_start, Theta[0] == theta_start]
    for i in range(N):
        constraints += [
            X[i+1] == X[i] + V[i] * ca.cos(Theta[i]) * dt,
            Y[i+1] == Y[i] + V[i] * ca.sin(Theta[i]) * dt,
            Theta[i+1] == Theta[i] + V[i] * ca.tan(Delta[i]) / L * dt,
            V[i+1] == V[i] + A[i] *dt,
            A[i] >= a_min, A[i] <= a_max,  # Acceleration constraints
            V[i] >= v_min, V[i] <= v_max,
            Delta[i] >= delta_min, Delta[i] <= delta_max  # Steering angle constraints
        ]

    if setup_type == 'back':
    # Obstacle avoidance constraints       
        for obstacle in obstacles:
            ox, oy, ow, oh = obstacle
            for i in range(N+1):
                constraints.append(ca.sqrt((X[i] - ox)**2 + (Y[i] - oy)**2) >= ow) #np.sqrt(ow ** 2 + oh ** 2

    # Set up the optimization problem
    opti.minimize(objective)
    opti.subject_to(constraints)

    # Provide an initial guess
    opti.set_initial(X, np.linspace(x_start, x_goal, N+1))
    opti.set_initial(Y, np.linspace(y_start, y_goal, N+1))
    opti.set_initial(Theta, np.linspace(theta_start, theta_goal, N+1))
    opti.set_initial(V, 0)
    opti.set_initial(Delta, 0)
    opti.set_initial(A, 0)

    # Set solver options for debugging
    opti.solver('ipopt', {'ipopt': {'print_level': 0, 'tol': 1e-6}})

    try:
        solution = opti.solve()
        # Extract and return the first control inputs from the solution
        v_opt = solution.value(V[0])
        delta_opt = solution.value(Delta[0])
        a_opt = solution.value(A[0])
        return v_opt, delta_opt,a_opt
    except Exception as e:
        print("Solver encountered an error:", e)
        return None, None  # Return a tuple of None to indicate failure

def real_time_mpc_loop():
    """
    TODO
    1. get the current vehicle state from sensors or communication interface
    2. get the obastacles' pos and dim
    3. Give control singal to the vehicle

    """
    # Initial vehicle state
    x_current, y_current, theta_current = []

    # Goal state
    x_goal, y_goal, theta_goal = []

    # Back waypoint
    x_back, y_back, theta_back = x_goal, y_goal + 5.5, theta_goal

    # Simulation parameters
    N = 10
    dt = 0.2
    L = 2.0

    setup_type1 = 'front'
    setup_type2 = 'back'

    # List of obstacles
    obstacles = []

    # Initialize lists to store the vehicle's trajectory for plotting
    x_trajectory = [x_current]
    y_trajectory = [y_current]

    phase_flag = 1

    while True:
        # From intial position to back waypoint
        if phase_flag == 1:
            # Solve the MPC problem with the current state as the starting point
            v_opt, delta_opt,a_opt = setup_mpc(setup_type1,N, dt, L, x_current, y_current, theta_current, x_back, y_back, theta_back, obstacles)
            
            if v_opt == None or delta_opt == None:
                print("Solution doesn't exist.")
                break
            
            # TODO :Give control singal to the vehicle
            
            # TODO :Update the current state of the vehicle

            # Store the new position
            x_trajectory.append(x_current)
            y_trajectory.append(y_current)

            # Termination condition if the vehicle reaches the goal area
            if np.sqrt((x_current - x_back)**2 + (y_current - y_back)**2) <= 0.8:  # threshold for reaching the goal
                print("Back point reached.")
                phase_flag = 2
        
        # From waypoint to goal position
        elif phase_flag == 2:
            # Solve the MPC problem with the current state as the starting point
            v_opt, delta_opt,a_opt = setup_mpc(setup_type2,N, dt, L, x_current, y_current, theta_current, x_goal, y_goal, theta_goal, obstacles)
            
            if v_opt == None or delta_opt == None:
                print("Solution doesn't exist.")
                break

            # TODO :Give control singal to the vehicle
        
            # TODO :Update the current state of the vehicle

            # Store the new position
            x_trajectory.append(x_current)
            y_trajectory.append(y_current)

            # Termination condition if the vehicle reaches the goal area
            if np.sqrt((x_current - x_goal)**2 + (y_current - y_goal)**2) <= 0.2:  # threshold for reaching the goal
                print("Goal reached.")
                break
        
        # # Wait for the next control cycle
        # time.sleep(dt)





