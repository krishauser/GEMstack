import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# from ...knowledge.vehicle import dynamics
# from ...knowledge.vehicle import geometry

class Vehicle_Dynamics:
    def __init__(self):
        # Vehicle dynamics limits
        # v_max  = settings.get('vehicle.limits.max_speed')
        # ax_max = settings.get('vehicle.limits.max_longitudinal_acceleration')
        # ax_min = settings.get('vehicle.limits.min_longitudinal_acceleration')
        # ay_max = settings.get('vehicle.limits.max_lateral_acceleration')
        # max_steering_rate = settings.get('vehicle.limits.max_steering_rate')
        # wheelbase = settings.get('vehicle.geometry.wheelbase')
        # max_wheel_angle = settings.get('vehicle.geometry.max_wheel_angle')

        # max_steering_rate = steer2front(max_steering_rate)

        #hard coded, remove
        v_max = 11.0
        max_wheel_angle = 0.6108
        ax_max = 3.0
        ax_min =  -3.0
        ay_max = 3.0
        max_steering_rate = 0.2
        wheelbase = 2.5

        self.v_bounds = [0, v_max]
        self.theta_bounds = [-max_wheel_angle, max_wheel_angle]

        self.ax_bounds = [ax_min, ax_max]
        self.ay_bounds = [-ay_max, ay_max]

        self.omega_bounds = [-max_steering_rate, max_steering_rate]
        
    # Dynamics
    def x_dot(self, x, u):
        dx = x[3] * ca.sin(x[2])
        dy = x[3] * ca.cos(x[2])
        dtheta = u[1]
        dv = u[0]

        return ca.vertcat(dx, dy, dtheta, dv)


def traj_opt_direct_transcription(vehicle : Vehicle_Dynamics,
                        x0 : np.array, xf : np.array,
                        N : int,
                        dt : float):
    assert len(x0) == 4
    assert len(xf) == 4
    assert N >= 1
    assert dt > 0
    
    # casadi function
    x = ca.SX.sym('x',4)
    u = ca.SX.sym('u',2)

    dx = vehicle.x_dot(x, u)
    dynamic_constraints = [x + dt * dx]

    forward_dynamics = ca.Function('f', [x,u], dynamic_constraints)

    # Initial guess
    x_init = np.linspace(x0[0], xf[0], N+1)
    y_init = np.linspace(x0[1], xf[1], N+1)
    theta_init = np.zeros(N+1)
    v_init = np.ones(N+1) * 3

    u_guess = np.zeros((N, 2))
    x_guess = np.concatenate([x_init, y_init, theta_init, v_init])

    # optimization variables
    opti = ca.Opti()
    X = opti.variable(N+1, 4)
    U = opti.variable(N,2)

    def terminal_cost(x):
        return (x[0]-xf[0])**2+(x[1]-xf[1])**2
    # def cone_cost():
    #     #TODO
    
    #objective function
    objective_function = terminal_cost(X[-1,:]) #TODO  + cone cost + time
    
    opti.minimize(objective_function)

    # #set up the optimization variables, including T+1 state variables and T control variables
    # opti = casadi.Opti()
    # xtraj = opti.variable(T+1,5)
    # utraj = opti.variable(T,2)
    # def terminal_cost(x):
    #     return (x[0]-xtarget[0])**2+(x[1]-xtarget[1])**2 + (x[2]-xtarget[2])**2 + x[3]**2
    # def control_cost(u,i):
    #     #TODO: consider control costs here
    #     # print(u)
    #     # print(i)
    #     # print(u[0])
    #     return (u[0]**2 + u[1]**2)
    #     #return 0.0
    # def state_cost(x,i):
    #     #TODO: set up obstacle avoidance costs here
    #     d_safe = 0.5
    #     k = 1000
    #     px, py = x[0], x[1]  # Extract vehicle position
    #     # print(px)
    #     cost = 0.0
    #     for obs in dubins.obstacles:
    #         center = obs.centroid
    #         boundary_point = list(obs.exterior.coords)[0]
    #         radius = center.distance(shapely.geometry.Point(boundary_point))
    #         # print(radius)
    #         d = casadi.sqrt((px - center.x)**2 + (py - center.y)**2)
    #         # print(d)
    #         cost += casadi.if_else(
    #             d-radius < d_safe,
    #             k * (1/(d-radius) - 1/d_safe)**2,
    #             0.0
    #         )
    #         # if d-radius < d_safe:
    #         #     print(d)
    #         # cost += k * (1/(d-radius) - 1/d_safe)**2  # Add penalty if too close
    #     return cost
    #     #return 0.0
    # def running_cost(x,u,i):
    #     return control_cost(u,i) + state_cost(x,i)
    # #setup objective function
    # obj = terminal_cost(xtraj[T,:])
    # for i in range(T):
    #     obj = obj + dt*running_cost(xtraj[i,:],utraj[i,:],i)
    # opti.minimize(obj)


    # initial state
    opti.subject_to(X[0,:] == x0.reshape((1,4)))

    # dynamics bounds
    for i in range(N):
        opti.subject_to(X[i+1,:] == forward_dynamics(X[i,:],U[i,:]).T)

    # dynamic limit bounds
    for i in range(N):
        opti.subject_to(opti.bounded(vehicle.theta_bounds[0],X[i+1,2],vehicle.theta_bounds[1]))
        opti.subject_to(opti.bounded(vehicle.v_bounds[0],X[i+1,3],vehicle.v_bounds[1]))
        opti.subject_to(opti.bounded(vehicle.ax_bounds[0],U[i,0],vehicle.ax_bounds[1]))
        opti.subject_to(opti.bounded(vehicle.omega_bounds[0],U[i,1],vehicle.omega_bounds[1]))

    opts = {
        'expand': True,
        'ipopti.max_iter': 1000,
    }
    p_opts = {"expand":True}
    s_opts = {"max_iter": 1000}
    opti.solver("ipopt",p_opts,
                        s_opts)
    
    for i  in range(N):
        opti.set_initial(X[i,:],x_guess[i])
        opti.set_initial(U[i,:],u_guess[i])

    sol = opti.solve()

    print("Terminal cost",'%.3f'%terminal_cost(sol.value(X)[N,:]))
    return (sol.value(X),sol.value(U),sol.value(objective_function))

def test_1():
    vehicle = Vehicle_Dynamics()
    x0 = np.array([0,0,0,0])
    xf = np.array([15,20,0,0])
    N = 100
    dt = 0.1
    X, U, _ = traj_opt_direct_transcription(vehicle=vehicle, x0=x0, xf=xf, N=N, dt=dt)

    # Plot
    plt.figure(figsize=(10,5))
    plt.plot(X[:, 0], X[:, 1], 'b-', label='Trajectory')
    plt.plot(x0[0],x0[1], 'x')
    plt.plot(xf[0],xf[1], 'x')
    # for i, (cx, cy) in enumerate(cone_positions):
    #     color = 'r' if i % 2 == 0 else 'g'
    #     plt.plot(cx, cy, 'o', color=color, markersize=10)
    #     plt.gca().add_patch(plt.Circle((cx, cy), cone_radius, color=color, fill=False))
    # plt.title(f'Slalom Trajectory (T = {T_opt:.2f}s)')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.axis('equal')
    plt.grid()
    plt.legend()


    # Time vector
    t = np.linspace(0, N, N+1)

    # Plot states
    x_labels = ['x', 'y', 'theta', 'v']
    u_labels = ['a', 'omega']
    plt.figure(figsize=(12, 8))
    for i in range(4):
        plt.subplot(6, 1, i+1)
        plt.plot(t, X[:, i])
        plt.ylabel(x_labels[i])
        plt.grid(True)
    for i in range(2):
        plt.subplot(6, 1, i+5)
        plt.plot(t[:-1], U[:, i])
        plt.ylabel(u_labels[i])
        plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        test = sys.argv[1]
    if test.startswith('test_1'):
        test_1()
    



