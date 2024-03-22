from typing import List, Tuple
from ..component import Component
from ...utils import serialization
from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph
import os
import numpy as np

from itertools import product
from common import SetQueue, GridMap, tic, toc, limit_angle
import math
from copy import deepcopy


# 栅格化位置和方向
MAP_NORM = 1.0          # 地图一个像素表示多少米 (m/1) #! BUG MAP_NORM不为 1 时绘图鸡哥背景错位
YAW_NORM = math.pi / 6  # 每多少rad算同一个角度 (rad/1)

# # 起点终点设置
# START = [5.0, 35.0, -math.pi/6] # 起点 (x, y, yaw), y轴向下为正, yaw顺时针为正
# END = [115.0, 60.0, math.pi/2]  # 终点 (x, y, yaw), y轴向下为正, yaw顺时针为正
ERR = 0.5                       # 与终点距离小于 ERR 米时停止搜索

# 车辆模型
CAR_LENGTH = 4.5                   # 车辆长度 (m)
CAR_WIDTH = 2.0                    # 车辆宽度 (m)
CAR_MAX_STEER = math.radians(30)   # 最大转角 (rad)
CAR_MAX_SPEED = 8                  # 最大速度 (m/s)

def check_polygon_collision(polygon1, polygon2):
    def project_polygon(axis, polygon):
        min_projection = float('inf')
        max_projection = float('-inf')
        for vertex in polygon:
            projection = axis[0] * vertex[0] + axis[1] * vertex[1]
            min_projection = min(min_projection, projection)
            max_projection = max(max_projection, projection)
        return min_projection, max_projection

    def is_separating_axis(axis, polygon1, polygon2):
        min1, max1 = project_polygon(axis, polygon1)
        min2, max2 = project_polygon(axis, polygon2)
        return max1 < min2 or max2 < min1

    edges = []
    for polygon in [polygon1, polygon2]:
        for i in range(len(polygon)):
            edge = (polygon[i][0] - polygon[(i + 1) % len(polygon)][0],
                    polygon[i][1] - polygon[(i + 1) % len(polygon)][1])
            normal = (-edge[1], edge[0])
            edges.append(normal)

    for axis in edges:
        if is_separating_axis(axis, polygon1, polygon2):
            return False
    return True


# 定义运动模型
def motion_model(s, u, dt):
    """
    >>> u = [v, δ]
    >>> dx/dt = v * cos(θ)
    >>> dy/dt = v * sin(θ)
    >>> dθ/dt = v/L * tan(δ)
    """
    s = deepcopy(s)
    s[0] += u[0] * math.cos(s[2]) * dt
    s[1] += u[0] * math.sin(s[2]) * dt
    s[2] += u[0]/CAR_LENGTH * math.tan(u[1]) * dt
    s[2] = limit_angle(s[2])
    return s




# 坐标节点
@dataclass(eq=False)
class HybridNode:
    """节点"""

    x: float
    y: float
    yaw: float

    G: float = 0.        # G代价
    cost: float = None   # F代价
    parent: "HybridNode" = None # 父节点指针

    def __post_init__(self):
        # 坐标和方向栅格化
        self.x_idx = round(self.x / MAP_NORM) # int向下取整, round四舍五入
        self.y_idx = round(self.y / MAP_NORM)
        self.yaw_idx = round(self.yaw / YAW_NORM)
        if self.cost is None:
            self.cost = self.calculate_heuristic([self.x, self.y], END)
    
    def __call__(self, u, dt):
        # 生成新节点 -> new_node = node(u, dt)
        x, y, yaw = motion_model([self.x, self.y, self.yaw], u, dt)
        G = self.G + self.calculate_distance([self.x, self.y], [x, y]) + abs(yaw - self.yaw)
        return HybridNode(x, y, yaw, G, parent=self)
        
    def __eq__(self, other: "HybridNode"):
        # 节点eq比较 -> node in list
        return self.x_idx == other.x_idx and self.y_idx == other.y_idx and self.yaw_idx == other.yaw_idx
        #return self.__hash__() == hash(other)
        
    def __le__(self, other: "HybridNode"):
        # 代价<=比较 -> min(open_list)
        return self.cost <= other.cost
    
    def __lt__(self, other: "HybridNode"):
        # 代价<比较 -> min(open_list)
        return self.cost < other.cost
    
    def __hash__(self) -> int:
        # 节点hash比较 -> node in set
        return hash((self.x_idx, self.y_idx, self.yaw_idx))
       
    def heuristic(self, TARG = END):
        """启发搜索, 计算启发值H并更新F值"""
        H = self.calculate_heuristic([self.x, self.y], TARG)
        self.cost = self.G + H
        return H

    def is_end(self, err = ERR):
        """是否终点, 启发值H小于err"""
        if self.cost - self.G < err:
            return True
        return False
    
    def in_map(self, map_array = MAP.map_array):
        """是否在地图中"""
        return (0 <= self.x < map_array.shape[1]) and (0 <= self.y < map_array.shape[0]) # h*w维, 右边不能取等!!!

    def is_collided(self, all_state: AllState):
        """Check if the node is in collision with any agent in the scene."""
        # Calculate the bounding box of the current vehicle
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        half_length = CAR_LENGTH / 2
        half_width = CAR_WIDTH / 2
        corners = [
            (self.x + half_length * cos_yaw - half_width * sin_yaw, self.y + half_length * sin_yaw + half_width * cos_yaw),
            (self.x - half_length * cos_yaw - half_width * sin_yaw, self.y - half_length * sin_yaw + half_width * cos_yaw),
            (self.x - half_length * cos_yaw + half_width * sin_yaw, self.y - half_length * sin_yaw - half_width * cos_yaw),
            (self.x + half_length * cos_yaw + half_width * sin_yaw, self.y + half_length * sin_yaw - half_width * cos_yaw)
        ]

        # Check collision with each agent
        for agent_id, agent_state in all_state.agents.items():
            agent_corners = [
                (agent_state.pose.x + half_length * cos_yaw - half_width * sin_yaw, agent_state.pose.y + half_length * sin_yaw + half_width * cos_yaw),
                (agent_state.pose.x - half_length * cos_yaw - half_width * sin_yaw, agent_state.pose.y - half_length * sin_yaw + half_width * cos_yaw),
                (agent_state.pose.x - half_length * cos_yaw + half_width * sin_yaw, agent_state.pose.y - half_length * sin_yaw - half_width * cos_yaw),
                (agent_state.pose.x + half_length * cos_yaw + half_width * sin_yaw, agent_state.pose.y + half_length * sin_yaw - half_width * cos_yaw)
            ]

            if check_polygon_collision(corners, agent_corners):
                return True
        return False

        
    @staticmethod
    def calculate_distance(P1, P2):
        """欧氏距离"""
        return math.hypot(P1[0] - P2[0], P1[1] - P2[1])
    
    @classmethod
    def calculate_heuristic(cls, P, TARG):
        """启发函数"""
        return cls.calculate_distance(P, TARG)       # 欧式距离
        #return abs(P[0]-TARG[0]) + abs(P[1]-TARG[1]) # 曼哈顿距离

# 混合A*算法
class HybridAStar:
    """混合A*算法"""

    def __init__(self, num_speed=3, num_steer=3, move_step=2, dt=0.2,start=[0,0,0],end=[0,0,0], all_state=None):
        """混合A*算法

        Parameters
        ----------
        num_speed : int
            控制量 v 离散个数, num>=1
        num_steer : int
            控制量 δ 离散个数, num>=2
        move_step : int
            向后搜索的次数
        dt : float
            决策周期
        """

        # 起点
        self.start = HybridNode(*start) # 起点
        self.start.heuristic()          # 更新 F 代价
        self.all_state = all_state      # all_state 变量
        # Error Check
        end = HybridNode(*end)
        # if not self.start.in_map() or not end.in_map():
        #     raise ValueError(f"x坐标y坐标超出地图边界")
        if self.start.is_collided():
            raise ValueError(f"起点x坐标或y坐标在障碍物上")
        if end.is_collided():
            raise ValueError(f"终点x坐标或y坐标在障碍物上")
       
        # 算法初始化
        self.reset(num_speed, num_steer, move_step, dt)
        

    def reset(self, num_speed=3, num_steer=3, move_step=2, dt=0.2):
        """重置算法"""
        self.__reset_flag = False
        assert num_steer > 1, "转向离散个数必须大于1"
        self.u_all = [
            np.linspace(CAR_MAX_SPEED, 0, num_speed) if num_speed > 1 else np.array([CAR_MAX_SPEED]),
            np.linspace(-CAR_MAX_STEER, CAR_MAX_STEER, num_steer),
        ]
        self.dt = dt
        self.move_step = move_step
        self.close_set = set()                    # 存储已经走过的位置及其G值 
        self.open_queue = SetQueue()              # 存储当前位置周围可行的位置及其F值
        self.path_list = []                       # 存储路径(CloseList里的数据无序)


    def search(self):
        """搜索路径"""
        return self.__call__()

    
    def _update_open_list(self, curr: HybridNode):
        """open_list添加可行点"""
        for v, delta in product(*self.u_all):
            # 更新节点
            next_ = curr
            for _ in range(self.move_step):
                next_ = next_([v, delta], self.dt) # x、y、yaw、G_cost、parent都更新了, F_cost未更新
            
            # # 新位置是否在地图外边
            # if not next_.in_map():
            #     continue

            # 新位置是否碰到障碍物
            if next_.is_collided():
                continue
            # 新位置是否在 CloseList 中
            if next_ in self.close_set:
                continue

            # 更新F代价
            H = next_.heuristic()

            # open-list添加/更新结点
            self.open_queue.put(next_)
            
            # 当剩余距离小时, 走慢一点
            if H < 20:
                self.move_step = 1
                
            
    def __call__(self):
        """A*路径搜索"""
        assert not self.__reset_flag, "call之前需要reset"
        print("搜索中\n")

        # 初始化 OpenList
        self.open_queue.put(self.start)

        # 正向搜索节点
        tic()
        while not self.open_queue.empty():
            # 弹出 OpenList 代价 F 最小的点
            curr: HybridNode = self.open_queue.get()
            # 更新 OpenList
            self._update_open_list(curr)
            # 更新 CloseList
            self.close_set.add(curr)
            # 结束迭代
            if curr.is_end():
                break
        print("路径搜索完成\n")
        toc()

        # 节点组合成路径
        while curr.parent is not None:
            self.path_list.append([curr.x,curr.y])
            curr = curr.parent
        self.path_list.reverse()
            
        # 需要重置
        self.__reset_flag = True

        return self.path_list

class StaticRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, routefn : str, frame : str = 'start'):
        self.routefn = routefn
        base, ext = os.path.splitext(routefn)
        if ext in ['.json','.yml','.yaml']:
            with open(routefn,'r') as f:
                self.route = serialization.load(f)
        elif ext == '.csv':
            waypoints = np.loadtxt(routefn,delimiter=',',dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[:,:2]
            if frame == 'start':
                self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
            elif frame == 'global':
                self.route = Route(frame=ObjectFrameEnum.GLOBAL,points=waypoints.tolist())
            elif frame == 'cartesian':
                self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,points=waypoints.tolist())
            else:
                raise ValueError("Unknown route frame {} must be start, global, or cartesian".format(frame))
        else:
            raise ValueError("Unknown route file extension",ext)

    def state_inputs(self):
        return []

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    def update(self):
        return self.route
    
class NavigationRoutePlanner(Component):
    """Returns a straight line as the desired route."""
    def __init__(self, start : List[float], end : List[float]):
        # start and end are [x, y, yaw, speed, steer] in the START frame
        self.start = start
        self.end = end
        print("NavigationRoutePlanner: start",start)
        print("NavigationRoutePlanner: end",end)

    def state_inputs(self):
        # return ['vehicle', 'roadgraph']
        return ['all'] # Temporary for collision detection, should be replaced by the roadgraph

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    # def update(self, vehicle : VehicleState, roadgraph : Tuple[Roadmap,Roadgraph]):
    #     # TODO: Figure out what is a Roadgraph
    #     roadmap, roadgraph = roadgraph

    #     # TODO: make this a real route
    #     route = Route(frame=ObjectFrameEnum.START,points=[self.start[:2],self.end[:2]])
    #     return route
    
    def update(self, state : AllState):
        # We can use agents to detect collisions, just like hw3
        # TODO: Shoule be replaced by the roadgraph to follow the GEMstack decision-making graph
        agents = state.agents
        
        # TODO: make this a real route
        hybrid_astar = HybridAStar(num_speed=3, num_steer=3, move_step=2, dt=0.2, start=self.start[:3], end=self.end[:3], all_state=state)
        points = hybrid_astar.path_list
        
        route = Route(frame=ObjectFrameEnum.START,points=points)
        return route