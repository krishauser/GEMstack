from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment
import numpy as np
import importlib.util


'''
CONFIG FILE/PRARAMETERS for KalmanTracker:
(some examples for configs are provided/set as defaults. )

MUST PROVIDE:
1: dim_x = the dimensions for state
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

class KalmanTracker:
    def __init__(self, config_file_path=None, \
                 dim_x=4, dim_z=2, F=None, H=None, P= None, Q=None, R=None, \
                 max_age=6, cost_function=None, threshold=10, initial_state=None):
        
        def default_cost_fn(predicted_state, measurement):
            return np.linalg.norm(predicted_state[0:2] - measurement)
        
        def default_initial_state(measurement):
            state = np.zeros(dim_x)
            state[0:dim_z] = measurement
            return state
        

        if config_file_path is not None:
            spec = importlib.util.spec_from_file_location("config", config_file_path)
            config = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(config)
            
            self.kalman_filters = {}
            self.max_id = 0
                
            self.dim_x = config.dim_x
            self.dim_z = config.dim_z
            self.F = config.F
            self.H = config.H
            self.P = config.P
            self.Q = config.Q
            self.R = config.R
            self.max_age = config.max_age
            self.threshold = config.threshold
            self.cost_function = config.cost_function
            self.initial_state = config.initial_state
            
        else: # if no config file is provided, use the parameters provided OR defaults. 
            self.kalman_filters = {}
            self.max_id = 0
                
            self.dim_x = dim_x
            self.dim_z = dim_z
            self.F = np.array([[1, 0, 1, 0], 
                                [0, 1, 0, 1], 
                                [0, 0, 1, 0], 
                                [0, 0, 0, 1]]) if F is None else F
            self.H = np.array([[1, 0, 0, 0], 
                               [0, 1, 0, 0]]) if H is None else H
            self.P = np.eye(dim_x) * 100 if P is None else P
            self.Q = np.eye(dim_x) * 0.01 if Q is None else Q
            self.R = np.eye(dim_z) * 1 if R is None else R
            self.max_age = max_age
            self.threshold = threshold
            self.cost_function = default_cost_fn if cost_function is None else cost_function
            self.initial_state = default_initial_state if initial_state else initial_state
    
    
    # Must be called in loop to continuously update all tracked objects
    # bounding_boxes are just the list of measurements/observations/sensor readings
    def update_pedestrian_tracking(self, bounding_boxes):
    
        # Predict the next state for each pedestrian using past state
        predicted_states = {}
        for pedestrian_id, kalman_filter in self.kalman_filters.items():
            kalman_filter.predict()
            # kalman_filter.x now stores the prediction for the future state.
            predicted_states[pedestrian_id] = kalman_filter.x
        
        # Match observed bounding boxes with predicted future states
        cost_matrix, pedestrian_id_list = self.compute_cost_matrix(predicted_states, bounding_boxes)
        matches = self.compute_matching(cost_matrix, pedestrian_id_list)
        
        # Update matched pedestrians
        matched_pedestrians = set()
        matched_bboxes = set()
        for pedestrian_id, bbox_idx in matches.items():
            self.kalman_filters[pedestrian_id].update(bounding_boxes[bbox_idx])
            self.kalman_filters[pedestrian_id].time_since_update = 0
            matched_pedestrians.add(pedestrian_id)
            matched_bboxes.add(bbox_idx)
            
        # For unmatched Kalman filters, increase time since last update by 1
        for pedestrian_id in (set(self.kalman_filters.keys()) - matched_pedestrians):
            self.kalman_filters[pedestrian_id].time_since_update += 1
        self.delete_old_tracks()
        
        # For unmatched bboxes, create a new kalman filter
        for col_idx in (set(range(len(bounding_boxes))) - matched_bboxes):
            pedestrian_id = self.generate_new_pedestrian_id()
            self.kalman_filters[pedestrian_id] = self.create_kalman_filter(bounding_boxes[col_idx])
            matches[pedestrian_id]= col_idx
        # Return the tracked pedestrians (mapping pedestrian ID to state)
        tracked_pedestrians = {
            pedestrian_id: kalman_filter.x for pedestrian_id, kalman_filter in self.kalman_filters.items()
        }
        return tracked_pedestrians, matches
    
    ### HELPER FUNCTIONS
    def generate_new_pedestrian_id(self):
        self.max_id += 1
        return str(self.max_id)

    def delete_old_tracks(self):
        to_delete = []
        for pedestrian_id, kalman_filter in self.kalman_filters.items():
            if kalman_filter.time_since_update > self.max_age:
                to_delete.append(pedestrian_id)
        for pedestrian_id in to_delete:
            del self.kalman_filters[pedestrian_id]
            
    def compute_cost_matrix(self,predicted_states, bounding_boxes):
        cost_matrix = np.zeros((len(predicted_states), len(bounding_boxes)))
        ped_id_list = []
        for i, (ped_id, predicted_state) in enumerate(predicted_states.items()):
            ped_id_list.append(ped_id)
            for j, bounding_box in enumerate(bounding_boxes):
                cost_matrix[i, j] = self.cost_function(predicted_state, bounding_box)
        return cost_matrix, ped_id_list
    
    def compute_matching(self, cost_matrix, ped_id_list):
        row_indices, col_indices = linear_sum_assignment(cost_matrix)        
        # Maps pedestrian ID to observed box index
        matching = {}
        for i in range(len(row_indices)):
            # row_indices[i], col_indices[i] is matched according to the linear solver
            # but, only include matches that have enough similarity (less than threshold)
            if cost_matrix[row_indices[i], col_indices[i]] <= self.threshold:
                matching[ped_id_list[i]] = col_indices[i]
        return matching
    
    def create_kalman_filter(self, bounding_box):
        kalman_filter = KalmanFilter(dim_x=self.dim_x, dim_z=self.dim_z)
        # Initialize the state with the bounding box
        kalman_filter.x = self.initial_state(bounding_box)
        
        # State transition matrix
        kalman_filter.F = self.F.copy()
        
        # Measurement matrix
        kalman_filter.H = self.H.copy()
            
        # Initial state uncertainty
        kalman_filter.P = self.P.copy()
        
        # Process noise (uncertainty of process)
        kalman_filter.Q = self.Q.copy()
        
        # Measurement noise covariance (uncertainty of obtained measurements/bounding boxes)
        kalman_filter.R = self.R.copy()
        
        kalman_filter.time_since_update = 0
        return kalman_filter
