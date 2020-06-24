"""A demonstration of ensemble level dynamics for rendezvous.
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
err_threshold = 0.5
p = 2  # Planar dimensions

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
    x = np.array(x) 
    
    # Store a list of trajectories
    x_traj = []
    x_traj.append(x)
    err = rendezvous_error(x)
    
    # Ensemble level dynamics
    # Create the laplacian matrix
    lapl = np.ndarray((num_robots, num_robots))
    lapl.fill(-1)
    np.fill_diagonal(lapl, num_robots-1)

    # Store a list of trajectories
    x_traj = []
    x_traj.append(x)
    err = rendezvous_error(x)
    while err>err_threshold:
        # Calculate the point to move towards
        dx = np.kron(lapl, np.identity(p)) @ x.flatten()
        dx = dx.reshape((num_robots, 2))
        # Calculate direction to move towards
        theta = (np.arctan2(dx[:,1], dx[:,0]) + np.pi) % (2 * np.pi) - np.pi
        dist = np.hypot(dx[:,0], dx[:,1])
        x[:,0] = x[:,0] -  dist * np.cos(theta) * dt
        x[:,1] = x[:,1] -  dist * np.sin(theta) * dt
        
        err = rendezvous_error(x)
        plt.cla()
        for robot_pose in x:
            plt.scatter(robot_pose[0], robot_pose[1])
        plt.xlim(0, 20)
        plt.ylim(0, 20)
        plt.pause(0.1)
    plt.show()