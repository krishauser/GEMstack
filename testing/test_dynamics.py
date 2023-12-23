#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.knowledge.vehicle import dynamics
import matplotlib.pyplot as plt
import numpy as np

def test_pedal_to_accel():
    gear = 1
    #vels = [0.0,0.01,0.1,1.0,5.0,10.0]
    #pitches = [0.0]
    vels = [0.0,1.0,5.0,10.0]
    pitches = [0.0,-np.radians(10.0),np.radians(10.0)]
    for v in vels:
        for pitch in pitches:
            x = np.linspace(0.0,1.0,100)
            y = np.zeros(100)
            for i,p in enumerate(x):
                a = dynamics.pedal_positions_to_acceleration(p,0.0,v,pitch,gear)
                y[i] = a
            plt.plot(x,y,label="v %.2f m/s, pitch %.2f deg"%(v,np.degrees(pitch)))
    plt.title("Gear=1")
    plt.xlabel("Accelerator position")
    plt.ylabel("Acceleration m/s^2")
    plt.legend()
    plt.show()

    for v in vels:
        for pitch in pitches:
            x = np.linspace(0.0,1.0,100)
            y = np.zeros(100)
            for i,p in enumerate(x):
                a = dynamics.pedal_positions_to_acceleration(0.0,p,v,pitch,gear)
                y[i] = a
            plt.plot(x,y,label="v %.2f m/s, pitch %.2f deg"%(v,np.degrees(pitch)))
    plt.title("Gear=1")
    plt.xlabel("Brake position")
    plt.ylabel("Acceleration m/s^2")
    plt.legend()
    plt.show()

def test_accel_to_pedal():
    gear = 1
    #vels = [0.0,0.01,0.1,1.0,5.0,10.0]
    #pitches = [0.0]
    #vels = [0.0,1.0,5.0,10.0]
    vels = [0.0,1.0]
    pitches = [0.0,-np.radians(10.0),np.radians(10.0)]
    for v in vels:
        for pitch in pitches:
            x = np.linspace(-2.5,2.5,100)
            y1 = np.zeros(100)
            y2 = np.zeros(100)
            for i,a in enumerate(x):
                acc,brake,gear_des = dynamics.acceleration_to_pedal_positions(a,v,pitch,gear)
                y1[i] = acc
                y2[i] = brake
                if gear_des != gear:
                    print("For v %.2f m/s, pitch %.2f deg, a %.2f m/s^2, gear changed from %d to %d"%(v,np.degrees(pitch),a,gear,gear_des))
            plt.plot(x,y1,label="Acc v %.2f m/s, pitch %.2f deg"%(v,np.degrees(pitch)))
            plt.plot(x,y2,linestyle='--',label="Brake v %.2f m/s, pitch %.2f deg"%(v,np.degrees(pitch)))
    plt.title("Gear=1")
    plt.ylabel("Pedal position")
    plt.xlabel("Acceleration m/s^2")
    plt.legend()
    plt.show()

if __name__=='__main__':
    #test_pedal_to_accel()
    test_accel_to_pedal()