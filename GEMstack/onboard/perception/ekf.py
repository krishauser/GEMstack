#Implementation of Error State Kalman Filter
from ...mathutils.transforms import Quaternion, skew_symmetric
import numpy as np


class ESKF(object):
    def __init__(self, initial_p = np.array([0,0,0]), initial_v= np.array([0,0,0]), initial_q = np.array([1,0,0,0]), initial_p_cov=np.eye(9), initial_t=2.055):
        #TODO, read from calibration
        # self.state = initial_state
        self.p = initial_p
        self.v = initial_v
        self.q = initial_q

        self.p_cov = initial_p_cov
        self.time = initial_t   #the state is estimated at time

    def propagate(self, imu_f, imu_w, new_arrive_time):
        delta_t = new_arrive_time - self.time  #TODO, we need to fix header to this for std_msgs header, use rospy.time()
        # delta_t = 0.005
        # 1. Update nominal state with IMU inputs
        Rotation_Mat = Quaternion(*self.q).to_mat()
        new_p = self.p + delta_t * self.v + 0.5 * (delta_t ** 2) * (Rotation_Mat.dot(imu_f) + g)
        new_v = self.v + delta_t * (Rotation_Mat.dot(imu_f) - g)
        new_q = Quaternion(euler = delta_t * imu_w).quat_mult(self.q)

        # 1.1 Linearize Motion Model and compute Jacobians
        F = np.eye(9)
        imu = imu_f.reshape((3, 1))
        F[0:3, 3:6] = delta_t * np.eye(3)
        F[3:6, 6:9] = Rotation_Mat.dot(-skew_symmetric(imu)) * delta_t

        # 2. Propagate uncertainty
        Q = np.eye(6)
        Q[0:3, 0:3] = var_imu_f * Q[0:3, 0:3]
        Q[3:6, 3:6] = var_imu_w * Q[3:6, 3:6]
        Q = (delta_t ** 2) * Q #Integration acceleration to obstain Position
        new_p_cov = F.dot(self.p_cov).dot(F.T) + l_jac.dot(Q).dot(l_jac.T)

        self.p = new_p
        self.v = new_v
        self.q = new_q
        self.p_cov = new_p_cov
        self.time = data['imu_f'].t[arrive_index]


    #y_k is measured value from GNSS, in meters
    def measurement_update(self, sensor_var, y_k):
        p_cov_check = self.p_cov
        p_check = self.p
        v_check = self.v
        q_check = self.q

        # 3.1 Compute Kalman Gain
        R_cov = sensor_var * np.eye(3)
        K = p_cov_check.dot(h_jac.T.dot(np.linalg.inv(h_jac.dot(p_cov_check.dot(h_jac.T)) + R_cov)))    #R_cov is V

        # 3.2 Compute error state
        delta_x = K.dot(y_k - p_check)  #p_check is predicted measurement value

        # 3.3 Correct predicted state
        p_check = p_check + delta_x[:3]
        v_check = v_check + delta_x[3:6]
        q_check = Quaternion(axis_angle = delta_x[6:]).quat_mult(q_check)

        # 3.4 Compute corrected covariance
        p_cov_check = (np.eye(9) - K.dot(h_jac)).dot(p_cov_check)   #p_cov_check is P


        self.p = p_check
        self.v = v_check
        self.q = q_check
        self.p_cov = p_cov_check
        # return p_check, v_check, q_check, p_cov_check

