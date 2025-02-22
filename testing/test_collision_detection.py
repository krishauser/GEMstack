#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.onboard.planning.longitudinal_planning import CollisionDetector
import time

if __name__ == "__main__":
    # Vehicle parameters. x, y, theta (angle in radians)
    x1, y1, t1 = 4.0, 5.0, 0
    # Pedestrian parameters. x, y, theta (angle in radians)
    x2, y2, t2 = 15.0, 2.1, 0
    # Velocity vectors: [vx, vy]
    v1 = [0.1, 0]     # Vehicle speed vector
    v2 = [0, 0.5]     # Pedestrian speed vector
    # Total simulation time
    total_time = 10.0

    desired_speed = 0.5
    acceleration  = -0.5

    # Create and run the simulation.
    start_time = time.time()
    # Simulate with the above parameters: Whether to hit without decelerating
    sim = CollisionDetector(x1, y1, t1, x2, y2, t2, v1, v2, total_time, desired_speed, acceleration)

    collision_distance = sim.run(is_displayed=True)
    print(f"Collision distance: {collision_distance}")
    # print(f"Simulation took {time.time() - start_time:.3f} seconds.")