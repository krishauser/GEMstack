#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.knowledge.vehicle import geometry
from GEMstack.utils import settings
import matplotlib.pyplot as plt
import numpy as np

def test_steering_conversions():
    min_wheel_angle = settings.get('vehicle.geometry.min_wheel_angle')
    max_wheel_angle = settings.get('vehicle.geometry.max_wheel_angle')
    min_steering_angle = settings.get('vehicle.geometry.min_steering_angle')
    max_steering_angle = settings.get('vehicle.geometry.max_steering_angle')
    fig,axs = plt.subplots(1,2,figsize=(10,5))
    wheel = np.linspace(min_wheel_angle,max_wheel_angle,100)
    steer = [geometry.front2steer(w) for w in wheel]
    axs[0].plot(np.degrees(wheel),np.degrees(steer))
    axs[0].set_xlabel('wheel angle (deg)')
    axs[0].set_ylabel('steer angle (deg)')
    axs[0].set_title('front2steer')
    axs[0].grid(True)
    steer = np.linspace(min_steering_angle,max_steering_angle,100)
    wheel = [geometry.steer2front(s) for s in steer]
    axs[1].plot(np.degrees(steer),np.degrees(wheel))
    axs[1].set_xlabel('steer angle (deg)')
    axs[1].set_ylabel('wheel angle (deg)')
    axs[1].set_title('steer2front')
    axs[1].grid(True)
    plt.show()

if __name__=='__main__':
    test_steering_conversions()