import rospy
import rosbag

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

import numpy as np
import pcl
import os

T_livox2baselink = np.array([
    [ np.cos(0.314) , 0             , np.sin(0.314) , 0.56  ],
    [0              , 1             , 0             , 0     ],
    [-np.sin(0.314) , 0             , np.cos(0.314) , 1.82  ],
    [0              , 0             , 0             , 1     ]
])

T_ouster2baselink = np.array([
    [1, 0, 0, 0     ],
    [0, 1, 0, 0     ],
    [0, 0, 1, 2.23  ],
    [0, 0, 0, 1     ]
])









if __name__ == "__main__":
    
    NODE_NAME = 'lidar_filter'
    
    LIVOX_TOPIC = '/livox/lidar'  # type : sensor_msgs/PointCloud2
    OUSTER_TOPIC = '/ouster/points'  # type : sensor_msgs/PointCloud2
    
    rospy.init_node(NODE_NAME, anonymous=True)  # add unique number
    
    
    
    
    



