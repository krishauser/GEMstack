from string import Template

# 定义模板字符串
ego_template_string = '''
*****Ego States:*****
Current State:
 - Velocity (vx,vy): ($vx,$vy)
 - Heading Angular Velocity (v_yaw): ($v_yaw)
 - Acceleration (ax,ay): ($ax,$ay)
 - Can Bus: ($can_x,$can_y)
 - Heading Speed: ($heading_speed)
 - Steering: ($steering)
Historical Trajectory (last 2 seconds): $trajectory
Mission Goal: $mission_goal
'''

perception_template_string = '''
*****Perception Results:*****

Future trajectories for specific objects:
$objects

Distance to both sides of road shoulders of current ego-vehicle location:
Current ego-vehicle's distance to left shoulder is $distance_left m and right shoulder is $distance_right m
'''

commonsense_mem = '''
*****Traffic Rules:*****
- Avoid collision with other objects.
- Always drive on drivable regions.
- Avoid driving on occupied regions.
- Pay attention to your ego-states and historical trajectory when planning.
- Maintain a safe distance from the objects in front of you.
'''

experience_mem = '''
*****Past Driving Experience for Reference:*****
Most similar driving experience from memory with confidence score: 0.86:
The planned trajectory in this experience for your reference:
[(0.03,3.74), (0.02,7.68), (0.00,11.60), (-0.04,15.56), (-0.10,19.57), (-0.21,23.53)]
'''

reasoning = '''
*****Chain of Thoughts Reasoning:*****
Thoughts:
 - Notable Objects: car at (6.97,-24.46), moving to (6.04,-23.50) at 1.0 second
   Potential Effects: within the safe zone of the ego-vehicle at 1.0 second
 - Notable Objects: pedestrian at (7.19,16.00), moving to (7.22,16.71) at 1.0 second
   Potential Effects: within the safe zone of the ego-vehicle at 1.0 second
 - Notable Objects: car at (-3.70,47.92), moving to (-4.77,51.01) at 1.0 second
   Potential Effects: within the safe zone of the ego-vehicle at 1.0 second
Driving Plan: MOVE FORWARD WITH A CONSTANT SPEED
'''

def get_ego_prompts(ego_dict):
    # 创建一个 Template 对象
    template = Template(ego_template_string)
    filled_string = template.substitute(ego_dict)
    
    return filled_string

def get_perception_prompts(perception_dict):
    template = Template(perception_template_string)

    # 处理 objects 部分
    object_entries = []
    for obj in perception_dict['objects']:
        entry = f"Object type: {obj['type']}, object id: {obj['id']}, future waypoint coordinates in 3s: {obj['waypoints']}"
        object_entries.append(entry)
    objects_string = "\n".join(object_entries)

    # 整合所有替换数据
    full_data = {
        "objects": objects_string,
        "distance_left": perception_dict['distance_left'],
        "distance_right": perception_dict['distance_right']
    }
    
    # 填充模板
    filled_string = template.substitute(full_data)
    return filled_string

# 示例数据
ego_dict_sample = {
    "vx": "0.06", "vy": "3.51",
    "v_yaw": "0.00",
    "ax": "-0.06", "ay": "0.01",
    "can_x": "1.31", "can_y": "-0.06",
    "heading_speed": "3.52",
    "steering": "-0.15",
    "trajectory": "[(-0.31,-13.61), (-0.19,-10.33), (-0.11,-6.99), (-0.02,-3.50)]",
    "mission_goal": "FORWARD"
}
perception_dict_sample = {
    "objects": [
        {"type": "car", "id": 1, "waypoints": "[(6.97, -24.46), (6.04, -23.50), (5.07, -22.19), (4.29, -20.46), (3.45, -18.45), (2.92, -15.82)]"},
        {"type": "pedestrian", "id": 2, "waypoints": "[(7.19, 16.00), (7.22, 16.71), (7.25, 17.46), (7.25, 18.18), (7.26, 18.93), (7.23, 19.63)]"},
        {"type": "car", "id": 3, "waypoints": "[(-3.70, 47.92), (-4.77, 51.01), (-5.86, 53.97), (-7.08, 56.77), (-8.36, 59.23), (-9.98, 61.47)]"}
    ],
    "distance_left": "1.0",
    "distance_right": "4.0"
}
if __name__=='__main__':
    get_ego_prompts(ego_dict_sample)
    get_perception_prompts(perception_dict_sample)


def get_commonsense_mem():
    return commonsense_mem