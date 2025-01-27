#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import Path, ObjectFrameEnum
from GEMstack.onboard.planning.longitudinal_planning import longitudinal_plan,longitudinal_brake
import matplotlib.pyplot as plt
    
def test_longitudinal_planning():
    test_path = Path(ObjectFrameEnum.START,[(0,0),(1,0),(2,0),(3,0),(4,0),(5,0),(6,0)])
    test_path2 = Path(ObjectFrameEnum.START,[(5,0),(6,1),(7,2),(9,4)])

    test_traj = longitudinal_brake(test_path, 2.0, 0.0)
    assert (t1 < t2 for (t1,t2) in zip(test_traj.times[:-1],test_traj.times[1:]) )
    plt.plot(test_traj.times,[p[0] for p in test_traj.points])
    plt.title("Braking from 0 m/s (should just stay still)")
    plt.xlabel('time')
    plt.ylabel('position')
    plt.show()

    test_traj = longitudinal_brake(test_path, 2.0, 2.0)
    assert (t1 < t2 for (t1,t2) in zip(test_traj.times[:-1],test_traj.times[1:]) )
    plt.plot(test_traj.times,[p[0] for p in test_traj.points])
    plt.title("Braking from 2 m/s")
    plt.xlabel('time')
    plt.ylabel('position')
    plt.show()

    test_traj = longitudinal_plan(test_path, 1.0, 2.0, 3.0, 0.0)
    assert (t1 < t2 for (t1,t2) in zip(test_traj.times[:-1],test_traj.times[1:]) )
    plt.plot(test_traj.times,[p[0] for p in test_traj.points])
    plt.title("Accelerating from 0 m/s")
    plt.xlabel('time')
    plt.ylabel('position')
    plt.show()

    
    test_traj = longitudinal_plan(test_path, 1.0, 2.0, 3.0, 2.0)
    assert (t1 < t2 for (t1,t2) in zip(test_traj.times[:-1],test_traj.times[1:]) )
    plt.plot(test_traj.times,[p[0] for p in test_traj.points])
    plt.title("Accelerating from 2 m/s")
    plt.xlabel('time')
    plt.ylabel('position')
    plt.show()

    test_traj = longitudinal_plan(test_path, 0.0, 2.0, 3.0, 3.1)
    assert (t1 < t2 for (t1,t2) in zip(test_traj.times[:-1],test_traj.times[1:]) )
    plt.plot(test_traj.times,[p[0] for p in test_traj.points])
    plt.title("Keeping constant velocity at 3.1 m/s")
    plt.xlabel('time')
    plt.ylabel('position')
    plt.show()

    test_traj = longitudinal_plan(test_path, 2.0, 2.0, 20.0, 10.0)
    assert (t1 < t2 for (t1,t2) in zip(test_traj.times[:-1],test_traj.times[1:]) )
    plt.plot(test_traj.times,[p[0] for p in test_traj.points])
    plt.title("Too little time to stop, starting at 10 m/s")
    plt.xlabel('time')
    plt.ylabel('position')
    plt.show()

    test_traj = longitudinal_brake(test_path, 2.0, 10.0)
    plt.plot(test_traj.times,[p[0] for p in test_traj.points])
    plt.title("Too little time to stop, braking at 10 m/s")
    plt.xlabel('time')
    plt.ylabel('position')
    plt.show()

    test_traj = longitudinal_plan(test_path2, 1.0, 2.0, 3.0, 0.0)
    plt.plot(test_traj.times,[p[0] for p in test_traj.points])
    plt.title("Nonuniform planning")
    plt.xlabel('time')
    plt.ylabel('position')
    plt.show()



if __name__ == '__main__':
    test_longitudinal_planning()
