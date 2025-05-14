import numpy as np
import rospy
from septentrio_gnss_driver.msg import INSNavGeod
from ...state import ObjectPose, ObjectFrameEnum
from scipy.stats import chi2

class GNSSKalmanFilter:
    def __init__(self):
        self.is_initialized = False
        self.last_time = None
        self.state = None  
        self.covariance = None  
        self.H = np.zeros((6, 12))
        self.H[0, 0] = 1  # x
        self.H[1, 2] = 1  # y
        self.H[2, 4] = 1  # z
        self.H[3, 6] = 1  # yaw
        self.H[4, 8] = 1  # roll
        self.H[5, 10] = 1  # pitch

        self.Q = np.eye(12) * 0.1
        self.R = np.eye(6) * 0.1

        
        self.speed = None #TODO

    def update(self, time: float, state: ObjectPose = None, speed = None, th = 0) -> ObjectPose:
        if not self.is_initialized:
            if state is None:
                return None
            else:
                self._initialize(state, speed, time)
                return self._current_pose()

        dt = time - self.last_time
        if dt < 0:
            dt = 0  

        # Predict step
        F = self._compute_F(dt)
        self.state = F @ self.state
        self.covariance = F @ self.covariance @ F.T + self.Q

        # Update step if measurement is available
        if state is not None:
            z = np.array([state.x, state.y, state.z, state.yaw, state.roll, state.pitch])
            y = z - self.H @ self.state
            S = self.H @ self.covariance @ self.H.T + self.R
            S_inv = np.linalg.inv(S)
            mahalanobis_sq = y.T @ S_inv @ y

            # Calculate chi-squared critical value for given confidence
            dof = len(z)
            critical_value = chi2.ppf(th, dof)

            if mahalanobis_sq <= critical_value:
                # Proceed with update
                K = self.covariance @ self.H.T @ S_inv
                self.state += K @ y
                self.covariance = (np.eye(12) - K @ self.H) @ self.covariance

        self.last_time = time
        return self._current_pose()

    def _initialize(self, state: ObjectPose, speed: float, time: float):
        self.state = np.zeros(12)
        self.state[0] = state.x
        self.state[2] = state.y
        self.state[4] = state.z
        self.state[6] = state.yaw
        self.state[8] = state.roll
        self.state[10] = state.pitch

        self.speed = speed #TODO

        self.covariance = np.eye(12) * 0.1
        for i in [1, 3, 5, 7, 9, 11]:
            self.covariance[i, i] = 10.0  # Higher uncertainty for velocities

        self.is_initialized = True
        self.last_time = time

    def _compute_F(self, dt: float) -> np.ndarray:
        F = np.eye(12)
        indices = [0, 2, 4, 6, 8, 10]
        for i in indices:
            F[i, i + 1] = dt
        return F

    def _current_pose(self) -> ObjectPose:
        return ObjectPose(
            ObjectFrameEnum.GLOBAL,
            t=self.last_time,
            x=self.state[0],
            y=self.state[2],
            z=self.state[4],
            yaw=self.state[6],
            roll=self.state[8],
            pitch=self.state[10]
        ),self.speed

def test0():
    th = 0.95  # 95% confidence threshold

    # Test scenario: Car starting at position, making a right turn while climbing a hill
    # Timeline (seconds):
    # 0-2: Initialization and straight movement
    # 3-5: Right turn with roll
    # 6-8: Hill climb with pitch
    # 9: Invalid outlier position
    # 10-12: Normal operation continues

    # Initial position (London coordinates)
    print("-- Initialization --")
    print("Update 0:", kf.update(0, ObjectPose(ObjectFrameEnum.GLOBAL,
        0, -0.118092, 51.509865, 35.0, 0, 0, 0
    ), 0.00017,th=th))

    # Straight movement north at 60 km/h (~16.67 m/s)
    print("\n-- Straight movement --")
    for t in range(1, 3):
        print(f"Predict {t}:", kf.update(t))  # Prediction only
        
    # Valid update (2 seconds of movement)
    print("\nUpdate 3:", kf.update(3, ObjectPose(ObjectFrameEnum.GLOBAL,
        3,
        -0.118092 + 0.00003,  # Minimal longitude change
        51.509865 + 0.00048,  # ~53 meters north
        35.2,  # Small altitude gain
        5.0,   # Slight right turn
        1.5,    # Small roll from turning
        0.5     # Small pitch from acceleration
    ), 0.00017,th=th))

    # Turning right sequence with realistic coordinate changes
    print("\n-- Right turn sequence --")
    for t in range(4, 6):
        print(f"Update {t}:", kf.update(t, ObjectPose(ObjectFrameEnum.GLOBAL,
            t,
            -0.118092 + 0.00003 + 0.00001*(t-3),
            51.509865 + 0.00048 + 0.0001*(t-3),
            35.2 + 0.1*(t-3),
            5.0 + 15*(t-3),  # Increasing yaw
            1.5 + 0.5*(t-3),  # Roll increasing
            0.5 - 0.2*(t-3)   # Pitch decreasing
        ), 0.00017,th=th))

    # Hill climb with pitch changes
    print("\n-- Hill climb --")
    print("Predict 6:", kf.update(6))  # Prediction
    print("Update 7:", kf.update(7, ObjectPose(ObjectFrameEnum.GLOBAL,
        7,
        -0.118092 + 0.00006,
        51.509865 + 0.00098,
        36.5,   # Higher altitude
        35.0,   # Completed turn
        2.0,    # Reduced roll
        4.0     # Significant pitch from hill
    ), 0.00017,th=th))

    # Test outlier rejection (sudden jump to China coordinates)
    print("\n-- Outlier test --")
    print("Predict 8:", kf.update(8))  # Prediction
    print("Update 9 (OUTLIER):", kf.update(9, ObjectPose(ObjectFrameEnum.GLOBAL,
        9,
        116.397458,  # Beijing longitude
        39.909042,   # Beijing latitude
        50.0,        # Large altitude jump
        35.0,        # Same yaw
        2.0,
        4.0
    ), 0.00017,th=th))

    # Continue normal operation
    print("\n-- Post-outlier recovery --")
    for t in range(10, 13):
        print(f"Update {t}:", kf.update(t, ObjectPose(ObjectFrameEnum.GLOBAL,
            t,
            -0.118092 + 0.00006 + 0.00001*(t-9),
            51.509865 + 0.00098 + 0.0001*(t-9),
            36.5 + 0.1*(t-9),
            35.0 + 2*(t-9),
            2.0 - 0.3*(t-9),
            4.0 - 0.5*(t-9)
        ), 0.00017,th=th))

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 3:
        print("usage: python3 -m GEMstack.onboard.perception.gnss_kalman_filter [topic [threshold]]")
    kf = GNSSKalmanFilter()
    th = 1
    
    if len(sys.argv) ==1:#hard written test
        test0()
        exit()
        
    from time import sleep
    import math

    topic = sys.argv[1]
    if len(sys.argv) >= 3:
        th = float(sys.argv[2])
        
    def callback(msg: INSNavGeod):
        op = ObjectPose(ObjectFrameEnum.GLOBAL,
            t=rospy.get_time(),
            x=math.degrees(msg.longitude),   
            y=math.degrees(msg.latitude),
            z=msg.height,
            yaw=math.radians(msg.heading), 
            roll=math.radians(msg.roll),
            pitch=math.radians(msg.pitch),
            )
        kf.update(op.t,op,th)

    sub = rospy.Subscriber(topic, INSNavGeod, callback)
    while 1:
        sleep(1)
        print(kf.update(rospy.get_time()))
    

