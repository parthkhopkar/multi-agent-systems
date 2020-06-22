"""A demonstration of basic gradient descent flow rendezvous.
This code assumes that the communication graph is static and fully connected.
The robots simply move in the direction of negative gradient.
"""
import matplotlib.pyplot as plt
import numpy as np
from random import random
import time

# Define simulation parameters
xlim = 20
ylim = 20
num_robots = 3
dt = 0.01

def rendezvous_error(x):
    # Calculate the error
    err = 0
    for i in range(len(x)):
        for j in range(len(x)):
            if j == i:
                continue
            err += np.linalg.norm(x[i] - x[j])
    
    err /=2
    return err

if __name__ == '__main__':
    # Initialize pose for specified number of robots
    x = []  # System of robots with (X, Y, theta)

    for _ in range(num_robots):
        X_start = round(xlim * random(), 2)
        Y_start = round(ylim * random(), 2)
        # theta_start = round(2 * np.pi * random() - np.pi, 2)
        x.append(np.array([X_start, Y_start]))
        # plt.scatter(X_start, Y_start)

    # Store a list of trajectories
    x_traj = []
    x_traj.append(x)
    err = rendezvous_error(x)
    while err>0.5:
        # Create x_diff list which stores difference between current pose and desired pose
        x_diff = []
        for i in range(num_robots):
            x_diff.append(np.array([0.,0.]))
        # Calculate the point to move towards
        for i in range(num_robots):
            for j in range(num_robots):
                if j == i:
                    continue
                x_diff[i] += x[i] - x[j]
            theta = (np.arctan2(x_diff[i][1], x_diff[i][0]) + np.pi) % (2 * np.pi) - np.pi
            dist = np.hypot(x_diff[i][0], x_diff[i][1])
            x[i][0] = x[i][0] -  dist * np.cos(theta) * dt
            x[i][1] = x[i][1] -  dist * np.sin(theta) * dt
        
        err = rendezvous_error(x)
        print(x)
        plt.cla()
        for robot_pose in x:
            plt.scatter(robot_pose[0], robot_pose[1])
        plt.xlim(0, 20)
        plt.ylim(0, 20)
        plt.pause(0.1)
    plt.show()