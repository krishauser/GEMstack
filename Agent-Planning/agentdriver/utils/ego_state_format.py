def ego_state_to_ego_sample(working_memory):
    vx = working_memory['ego_states'][0]*0.5
    vy = working_memory['ego_states'][1]*0.5
    v_yaw = working_memory['ego_states'][4]
    ax = working_memory['ego_hist_traj_diff'][-1, 0] - working_memory['ego_hist_traj_diff'][-2, 0]
    ay = working_memory['ego_hist_traj_diff'][-1, 1] - working_memory['ego_hist_traj_diff'][-2, 1]
    cx = working_memory['ego_states'][2]
    cy = working_memory['ego_states'][3]
    vhead = working_memory['ego_states'][7]*0.5
    steeling = working_memory['ego_states'][8]

    xh1 = working_memory['ego_hist_traj'][0][0]
    yh1 = working_memory['ego_hist_traj'][0][1]
    xh2 = working_memory['ego_hist_traj'][1][0]
    yh2 = working_memory['ego_hist_traj'][1][1]
    xh3 = working_memory['ego_hist_traj'][2][0]
    yh3 = working_memory['ego_hist_traj'][2][1]
    xh4 = working_memory['ego_hist_traj'][3][0]
    yh4 = working_memory['ego_hist_traj'][3][1]
    cmd_vec = working_memory['goal']
    right, left, forward = cmd_vec
    if right > 0:
        mission_goal = "RIGHT"
    elif left > 0:
        mission_goal = "LEFT"
    else:
        assert forward > 0
        mission_goal = "FORWARD"
    ego_dict_sample = {
        "vx": f"{vx:.2f}", "vy": f"{vy:.2f}",
        "v_yaw": f"{v_yaw:.2f}",
        "ax": f"{ax:.2f}", "ay": f"{ay:.2f}",
        "can_x": f"{cx:.2f}", "can_y": f"{cy:.2f}",
        "heading_speed": f"{vhead:.2f}",
        "steering": f"{steeling:.2f}",
        "trajectory": f"[({xh1:.2f},{yh1:.2f}), ({xh2:.2f},{yh2:.2f}), ({xh3:.2f},{yh3:.2f}), ({xh4:.2f},{yh4:.2f})]",
        "mission_goal": mission_goal
    }
    return ego_dict_sample
