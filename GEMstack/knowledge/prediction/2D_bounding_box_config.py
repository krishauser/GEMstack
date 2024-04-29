import numpy as np

## Config file for Kalman filtering for 2D bounding box prediction
## State has 6 dimensions (x,y,length,width, vel_x, vel_y)
## Observation/measurement is 2D bounding box [4 dimensions: x, y, length, width]

'''
CONFIG FILE for Kalman filter

MUST PROVIDE:
1: dim_x = the dimensions for state)

2: dim_z = the dimensions for measurement

3: F = state transition matrix. Must be shape (dim_x, dim_x)

4: H = measurement matrix. Must be shape (dim_z, dim_x)

5: P = Covariance matrix for initial observation. Represents uncertainty of initial state.
    Must be matrix of shape (dim_x, dim_x)
    
6: Q = Process Covariance Matrix. Represents uncertainty of process
    Must be matrix of shape (dim_x, dim_x)
    
7: R = Measurement Covariance Matrix. Represents uncertainty of measurement
    Must be matrix of shape (dim_z, dim_z)
    
8: max_age = After how many time_steps with no observation should we delete a kalman tracker.
    Must be positive int
    
9: cost_function(predicted_state, observation) =
    function which calculates cost (or dissimilarity) between a
    predicted state (size dim_x) and an observation (size dim_z)

10: threshold = max limit for whether we consider a predicted state and new observation a match

11: initial_state(measurement) = 
     function which provides initial state, given just the first observation
     measurement is vector of size (dim_z). Output is vector of size (dim_x)
'''


# state is vector with [x,y,l,w,vel_x, vel_y]
dim_x = 6
# measurements (sensor observations/bounding boxes) are just [x,y,l,w]
dim_z = 4

F = np.array([[1, 0, 0, 0, 1, 0],
            [0, 1, 0, 0, 0, 1],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]])

H = np.array([[1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0]])

P = np.eye(dim_x) * 100

Q = np.eye(dim_x) * 0.01

R = np.eye(dim_z) * 1

max_age = 4

# IoU must be at least 0.3 or higher to be considered a match
threshold = -0.6

# This cost function is IoU (intersection over Union) of 2D bounding boxes
# Higher the IoU (overlapping region), higher is the similarity
# Since this is a cost function (measurement of dissimilarity), we negate the IoU
def cost_function(predicted_state, measurement):
    rect1 = predicted_state[0:4]
    rect2 = measurement
    rect1_x1, rect1_y1 = rect1[0] - rect1[2]/2, rect1[1] - rect1[3]/2
    rect1_x2, rect1_y2 = rect1[0] + rect1[2]/2, rect1[1] + rect1[3]/2
    rect2_x1, rect2_y1 = rect2[0] - rect2[2]/2, rect2[1] - rect2[3]/2
    rect2_x2, rect2_y2 = rect2[0] + rect2[2]/2, rect2[1] + rect2[3]/2

    # Calculate the coordinates of the intersection rectangle
    inter_x1 = max(rect1_x1, rect2_x1)
    inter_y1 = max(rect1_y1, rect2_y1)
    inter_x2 = min(rect1_x2, rect2_x2)
    inter_y2 = min(rect1_y2, rect2_y2)

    # Calculate the area of intersection
    inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)

    # Calculate the area of union
    rect1_area = rect1[2] * rect1[3]
    rect2_area = rect2[2] * rect2[3]
    union_area = rect1_area + rect2_area - inter_area

    # Calculate the IoU
    iou = inter_area / union_area if union_area > 0 else 0
    return -iou

# given measurement of [x,y,l,w], set state to [x,y,l,w,0,0]
def initial_state(measurement):
    state = np.zeros(dim_x)
    state[0:dim_z] = measurement
    return state