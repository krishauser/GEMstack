def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    timestep = 50

    x_start = points[0][0]
    y_start = points[0][1]
    x_end = points[-1][0]
    y_end = points[-1][1]
    print("x_start, y_start = ",[x_start, y_start])
    print("x_end, y_end = ",[x_end, y_end])
    dis = math.sqrt((x_end - x_start)**2 + (y_end - y_start)**2)
    dis_x = x_end - x_start
    dis_y = y_end - y_start

    v_start = current_speed
    vx_start = (dis_x / dis) * current_speed
    vy_start = (dis_y / dis) * current_speed
    acc_x = (dis_x / dis) * acceleration
    acc_y = (dis_y / dis) * acceleration
    vx_max = (dis_x / dis) * max_speed
    vy_max = (dis_y / dis) * max_speed
    dec_x = (dis_x / dis) * (-deceleration)
    dec_y = (dis_y / dis) * (-deceleration)
    print("vx_start, vy_start = ",[vx_start, vy_start])
    print("acc_x, acc_y = ",[acc_x, acc_y])
    print("vx_max, vy_max = ",[vx_max, vy_max])
    print("dec_x, dec_y = ",[dec_x, dec_y])

    if acceleration == 0: 
        # assert False
        print("const speed mode")
        t_cons_v = dis / current_speed

        t_dec = 0.5 * (2*(dis_x)/vx_start - (0 - vx_start)/(dec_x))
        T = t_dec + (0 - vx_start) / (dec_x)

        
        x_dec = x_start + t_dec * vx_start
        y_dec = y_start + t_dec * vy_start

        t_constv_array = np.linspace(0, t_dec, timestep, dtype=float)
        x_constv_array = x_start + vx_start * t_constv_array
        y_constv_array = y_start + vy_start * t_constv_array

        t_dec_array = np.linspace(0, T - t_dec, timestep, dtype=float)
        x_dec_array = x_dec + t_dec_array * vx_max + 0.5 * (t_dec_array**2) * dec_x
        y_dec_array = y_dec + t_dec_array * vy_max + 0.5 * (t_dec_array**2) * dec_y



        traj_points = []
        traj_times = []

        for i in range(len(t_constv_array - 1)):
            traj_points.append([x_constv_array[i],y_constv_array[i]])
            traj_times.append(t_constv_array[i])

        for i in range(len(t_dec_array)):
            traj_points.append([x_dec_array[i], y_dec_array[i]])
            traj_times.append(t_dec_array[i] + t_dec)

        # traj_points = [(x_start,y_start), (x_end,y_end)]
        # traj_times = [times[0], t_cons_v]
        print("traj_points = ", traj_points)
    else: 
        print("trapezoidal mode")
        #Solving for trapezoidal velocity profile
        t_acc = (max_speed - current_speed) / acceleration
        x_acc = x_start + t_acc * vx_start + 0.5 * (t_acc**2) * acc_x
        y_acc = y_start + t_acc * vy_start + 0.5 * (t_acc**2) * acc_y
        T_minus_t_dec = (0 - max_speed) / (-deceleration)
        x_dec = x_end - T_minus_t_dec * vx_max - 0.5 * (T_minus_t_dec**2) * (dec_x)
        y_dec = y_end - T_minus_t_dec * vy_max - 0.5 * (T_minus_t_dec**2) * (dec_y)
        t_dec = (x_dec - x_acc) / vx_max + t_acc
        T = T_minus_t_dec + t_dec 

        t_acc_array = np.linspace(times[0], t_acc, timestep, dtype=float)
        x_acc_array = x_start + t_acc_array * vx_start + 0.5 * (t_acc_array**2) * acc_x
        y_acc_array = y_start + t_acc_array * vy_start + 0.5 * (t_acc_array**2) * acc_y
        t_constv_array = np.linspace(0, t_dec-t_acc, timestep, dtype=float)
        x_constv_array = x_acc + vx_max * t_constv_array
        y_constv_array = y_acc + vy_max * t_constv_array
        t_dec_array = np.linspace(0, T-t_dec, timestep, dtype=float)
        x_dec_array = x_dec + t_dec_array * vx_max + 0.5 * (t_dec_array**2) * dec_x
        y_dec_array = y_dec + t_dec_array * vy_max + 0.5 * (t_dec_array**2) * dec_y

        traj_points = []
        traj_times = []

        for i in range(len(t_acc_array)-1):
            traj_points.append([x_acc_array[i],y_acc_array[i]])
            traj_times.append(t_acc_array[i])

        for i in range(len(t_constv_array)-1):
            traj_points.append([x_constv_array[i],y_constv_array[i]])
            traj_times.append(t_constv_array[i] + t_acc)

        for i in range(len(t_dec_array)):
            traj_points.append([x_dec_array[i],y_dec_array[i]])
            traj_times.append(t_dec_array[i] + t_dec)




        # traj_points = [(x_start,y_start), (x_acc,y_acc), (x_dec,y_dec), (x_end,y_end)]
        # traj_times = [times[0], t_acc, t_dec, T]
        print("traj_points = ", traj_points)

        if (x_dec<=x_acc) and (x_dec<=x_acc):
            print("triangular mode")
          
            # Using Sympy to solve equations 
            # Define symbols
            vx_dec, t_dec, T, vy_dec = sympy.symbols('vx_dec t_dec T vy_dec')
            print(vx_dec, t_dec, T, vy_dec, vx_start, acc_x)

            # Define x equations
            equations_x = [
                vx_dec - vx_start - t_dec * acc_x,
                T - t_dec + (vx_dec / dec_x),
                vx_dec * T / 2 - (x_end - x_start)
            ]
            
            solutions_x = sympy.solve(equations_x, dict = True)
            print(solutions_x)
            
            #correct index = 1
            vx_dec_value = solutions_x[1][vx_dec]
            t_dec_value = solutions_x[1][t_dec]
            T_value = solutions_x[1][T]

            # Define y equations
            equations_y = [
                vy_dec - vy_start - t_dec_value * acc_y,
            ]

            solutions_y = sympy.solve(equations_y, dict = True)
            print(solutions_y)

            #correct index = 0
            vy_dec_value = solutions_y[0][vy_dec]

            print("vx_dec:", vx_dec_value)
            print("t_dec:", t_dec_value)
            print("T:", T_value)
            print("vy_dec:", vy_dec_value)
            print("Data type of t_dec_value:", type(t_dec_value))

            x_dec = x_start + t_dec_value * vx_start + 0.5 * (t_dec_value**2) * acc_x
            y_dec = y_start + t_dec_value * vy_start + 0.5 * (t_dec_value**2) * acc_y

            t_acc_array = np.linspace(float(times[0]), float(t_dec_value), timestep, dtype=float)
            x_acc_array = x_start + t_acc_array * vx_start + 0.5 * (t_acc_array**2) * acc_x
            y_acc_array = y_start + t_acc_array * vy_start + 0.5 * (t_acc_array**2) * acc_y
            t_dec_array = np.linspace(float(0), float(T_value-t_dec_value), timestep, dtype=float)
            x_dec_array = x_dec + t_dec_array * vx_dec_value + 0.5 * (t_dec_array**2) * dec_x
            y_dec_array = y_dec + t_dec_array * vy_dec_value + 0.5 * (t_dec_array**2) * dec_y

            traj_points = []
            traj_times = []

            for i in range(len(t_acc_array)-1):
                traj_points.append([x_acc_array[i], y_acc_array[i]])
                traj_times.append(t_acc_array[i])

            for i in range(len(t_dec_array)):
                traj_points.append([x_dec_array[i], y_dec_array[i]])
                traj_times.append(t_dec_array[i] + t_dec_value)

            # traj_points = [(x_start,y_start), (x_dec,y_dec), (x_end,y_end)]
            # traj_times = [times[0], t_dec_value, T_value]
            print("traj_points = ", traj_points)

            if (x_dec < 0) or (y_dec < 0):
                print("Too little time to stop")
                T_value = (0 - current_speed) / (-deceleration) 
                x_end = x_start + T_value * vx_start + 0.5 * (T_value**2) * dec_x
                y_end = y_start + T_value * vy_start + 0.5 * (T_value**2) * dec_y
                print("T:", T_value)

                t_dec_array = np.linspace(0, T_value, timestep, dtype=float)
                x_dec_array = x_start + t_dec_array * vx_start + 0.5 * (t_dec_array**2) * dec_x
                y_dec_array = y_start + t_dec_array * vy_start + 0.5 * (t_dec_array**2) * dec_y

                traj_points = []
                traj_times = []

                for i in range(len(t_dec_array)):
                    traj_points.append([x_dec_array[i], y_dec_array[i]])
                    traj_times.append(t_dec_array[i])

            
                # traj_points = [(x_start,y_start), (x_end,y_end)]
                # traj_times = [times[0], T_value]
                # print("traj_points = ", traj_points)
       
    trajectory = Trajectory(path.frame,traj_points,traj_times)
    return trajectory


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]
    timestep = 50

    x_start = points[0][0]
    y_start = points[0][1]
    x_end = points[-1][0]
    y_end = points[-1][1]
    print("x_start, y_start = ",[x_start, y_start])
    print("x_end, y_end = ",[x_end, y_end])
    dt = 0.05
    t_list = [times[0]]
    point_list = points[0]
    cur_pos = points[0][0]
    while current_speed > 0:
        current_speed = current_speed - deceleration *dt
        cur_pos += current_speed*dt
        point_list.append((cur_pos, 0))
        t_list.append(t_list[-1]+dt)

    trajectory = Trajectory(path.frame,point_list,t_list)
    return trajectory
    
    if current_speed == 0: 
        print("still mode")

        traj_points = []
        traj_times = []

        t_still_array = np.linspace(0, times[-1], timestep, dtype=float)

        for i in range(len(t_still_array)):
            traj_points.append([0, 0])
            traj_times.append(t_still_array[i])

    else:
        v_start = current_speed
        vx_start = ((x_end - x_start) / (math.sqrt((x_end - x_start)**2 + (y_end - y_start)**2))) * current_speed
        vy_start = ((y_end - y_start) / (math.sqrt((x_end - x_start)**2 + (y_end - y_start)**2))) * current_speed
        dec_x = ((x_end - x_start) / (math.sqrt((x_end - x_start)**2 + (y_end - y_start)**2))) * (-deceleration)
        dec_y = ((y_end - y_start) / (math.sqrt((x_end - x_start)**2 + (y_end - y_start)**2))) * (-deceleration)
        print("vx_start, vy_start = ",[vx_start, vy_start])
        print("dec_x, dec_y = ",[dec_x, dec_y])

        t_dec = (0 - current_speed) / (-deceleration)
        x_dec = x_start + t_dec * vx_start + 0.5 * (t_dec**2) * dec_x
        y_dec = y_start + t_dec * vy_start + 0.5 * (t_dec**2) * dec_y

        t_dec_array = np.linspace(0, t_dec, timestep, dtype=float)
        x_dec_array = x_start + t_dec_array * vx_start + 0.5 * (t_dec_array**2) * dec_x
        y_dec_array = y_start + t_dec_array * vy_start + 0.5 * (t_dec_array**2) * dec_y

        traj_points = []
        traj_times = []

        for i in range(len(t_dec_array)):
            traj_points.append([x_dec_array[i], y_dec_array[i]])
            traj_times.append(t_dec_array[i])

        
        # traj_points = [(x_start,y_start), (x_dec,y_dec)]
        # traj_times = [times[0], t_dec]
    trajectory = Trajectory(path.frame,traj_points,traj_times)
    return trajectory
