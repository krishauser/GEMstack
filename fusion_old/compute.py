from SensorFusion import *
np.set_printoptions(precision=4, suppress=False)

def testForwardBackward():
    print("Test Forward Backward Cvt")
    testTF = np.random.rand(6)
    print(f"testTF   : {testTF}")
    resultTF = cvtTransMat2TF2(cvtTF2TransMat(testTF))
    print(f"resultTF : {resultTF}")

CORNER_CAM_LOOK_DOWN = -20


FRONT_CAM_TF = cvtRotateMat2YawPitchRoll(
    rotate_z(-90 / 180 * np.pi) @ rotate_x(-90 / 180 * np.pi)
    # matrix order
        # fixed(2) fixed(1)
)

FRONT_LEFT_CAM_TF = cvtRotateMat2YawPitchRoll(
    rotate_z(45 / 180 * np.pi) \
    @ rotate_z(-90 / 180 * np.pi) @ rotate_x(-90 / 180 * np.pi) \
    @ rotate_x(CORNER_CAM_LOOK_DOWN / 180 * np.pi)
    # matrix order
        # fixed(3)
        # fixed(2) fixed(1)
        # euler(4)
)

FRONT_RIGHT_CAM_TF = cvtRotateMat2YawPitchRoll(
    rotate_z(-45 / 180 * np.pi) \
    @ rotate_z(-90 / 180 * np.pi) @ rotate_x(-90 / 180 * np.pi) \
    @ rotate_x(CORNER_CAM_LOOK_DOWN / 180 * np.pi)
    # matrix order
        # fixed(3)
        # fixed(2) fixed(1)
        # euler(4)
)

REAR_LEFT_CAM_TF = cvtRotateMat2YawPitchRoll(
    rotate_z(135 / 180 * np.pi) \
    @ rotate_z(-90 / 180 * np.pi) @ rotate_x(-90 / 180 * np.pi) \
    @ rotate_x(CORNER_CAM_LOOK_DOWN / 180 * np.pi)
    # matrix order
        # fixed(3)
        # fixed(2) fixed(1)
        # euler(4)
)

REAR_RIGHT_CAM_TF = cvtRotateMat2YawPitchRoll(
    rotate_z(-135 / 180 * np.pi) \
    @ rotate_z(-90 / 180 * np.pi) @ rotate_x(-90 / 180 * np.pi) \
    @ rotate_x(CORNER_CAM_LOOK_DOWN / 180 * np.pi)
    # matrix order
        # fixed(3)
        # fixed(2) fixed(1)
        # euler(4)
)


if __name__ == '__main__':
    
    # testForwardBackward()
    
    print(f"FRONT_CAM_TF       : {FRONT_CAM_TF}")
    print(f"FRONT_LEFT_CAM_TF  : {FRONT_LEFT_CAM_TF}")
    print(f"FRONT_RIGHT_CAM_TF : {FRONT_RIGHT_CAM_TF}")
    print(f"REAR_LEFT_CAM_TF   : {REAR_LEFT_CAM_TF}")
    print(f"REAR_RIGHT_CAM_TF  : {REAR_RIGHT_CAM_TF}")
    
    
