import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from slam.ekf_slam import predict, update, compute_H
from sensors.lidar_model import simulate_lidar_scan

# --- World Setup ---
walls = [
    ('h', 0.0), ('h', 4.0),
    ('v', 0.0), ('v', 4.0)
]
landmarks = [(0.0,0.0),(4.0,0.0),(0.0,4.0),(4.0,4.0)]

# --- Initial State ---
true_pose = np.array([2.0, 2.0, 0.0])
mu = np.array([2.0, 2.0, 0.0])
sigma = np.eye(3) * 0.1

# --- Noise Parameters ---
Q = np.eye(3) * 0.01
R = np.eye(2) * 0.1

# --- Motion Command (circle) ---
u = (0.05, 0.05)  # forward 0.1m, turn 0.05 rad

def move_true_robot(true_pose, u):
    d, delta_theta = u
    x, y, theta = true_pose
    # add small noise to simulate real world
    noisy_d = d + np.random.normal(0, 0.01)
    noisy_dtheta = delta_theta + np.random.normal(0, 0.005)
    
    x_new = x + noisy_d * np.cos(theta)
    y_new = y + noisy_d * np.sin(theta)
    x_new = np.clip(x_new, 0.1, 3.9)
    y_new = np.clip(y_new, 0.1, 3.9)
    theta_new = theta + noisy_dtheta
    return np.array([x_new, y_new, theta_new])

def measure_landmark(true_pose, landmark):
    x, y, theta = true_pose
    lx, ly = landmark
    dx = lx - x
    dy = ly - y
    distance =  np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx) - theta
    # add sensor noise
    return np.array([
        distance + np.random.normal(0, 0.1),
        angle + np.random.normal(0, 0.05)
    ])
def expected_measurement(mu, landmark):
    x, y, theta = mu
    lx, ly = landmark
    dx = lx - x
    dy = ly - y
    distance = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx) - theta
    return np.array([distance, angle])
# --- Storage for plotting ---
true_path = [true_pose.copy()]
ekf_path = [mu.copy()]

fig, ax = plt.subplots(figsize=(8,8))

def animate(frame):
    global true_pose, mu, sigma
    
    # 1. move true robot
    true_pose = move_true_robot(true_pose, u)
    
    # 2. EKF predict
    mu, sigma = predict(mu, sigma, u, Q)
    
    # 3. update with each landmark
    for landmark in landmarks:
        z = measure_landmark(true_pose, landmark)
        z_expected = expected_measurement(mu, landmark)
        innovation = z - z_expected
        H = compute_H(mu, landmark)
        mu, sigma = update(mu, sigma, innovation, H, R)
    
    # 4. store paths
    true_path.append(true_pose.copy())
    ekf_path.append(mu.copy())
    
    # 5. draw
    ax.clear()
    ax.set_xlim(-0.5, 4.5)
    ax.set_ylim(-0.5, 4.5)
    
    # YOUR CODE HERE - draw these three things:
    # a) true path in green
    # b) EKF estimated path in red  
    # c) landmarks as blue stars
    true_path_arr = np.array(true_path)
    ekf_path_arr = np.array(ekf_path)
    landmarks_arr = np.array(landmarks)

    ax.plot(true_path_arr[:,0], true_path_arr[:,1], 'g', label='True path')
    ax.plot(ekf_path_arr[:,0], ekf_path_arr[:,1],'r', label='EKF estimate')
    ax.plot(landmarks_arr[:,0],landmarks_arr[:,1],'b*',label='Landmarks')
    ax.plot(true_pose[0], true_pose[1], 'go', markersize=10)
    ax.plot(mu[0],        mu[1],        'ro', markersize=10)
    ax.legend()
    ax.set_title(f'Step {frame}')
    ax.grid(True)
ani = animation.FuncAnimation(fig, animate, frames=200, interval=50)
plt.show()