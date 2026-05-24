# 2D LiDAR SLAM from Scratch
Builds 2D LiDAR SLAM using pure mathematical foundations

## What This Project Is
This project applies 2D SLAM(Simultaneously Localization and Mapping) on robot
without using SLAM libraries.This uses prerecorded data to test the algorithm works correctly.

## What I Am Learning
- SLAM is challenging becuase it requires a robot to perform localization and mapping concurrently.
The robot should estimate its own position while building map of the environment. Both tasks depend on each other.
- A robot represents its position not as single point but as covariance matrix P.SLAM models it as probability distribution
showing the uncertainity in the robot's state.
- In the Extended Kalman Filter (EKF), the prediction steps increases uncertainity over time due to accumulated error.This growing uncertainity is represented in covariance matrix.
- The Kalman gain determines the weighted confidence assigned to sensor measuremnet versus the predicted state.If the uncertainity in prediction state is high, the filter relies more on sensor data and if the sensor mesauremnets are noisy , it trusts the predicted data more.
- EKF requires Jacobians because it operates on nonlinear system models. Linear KF equations can't be directly applied to nonlinear functions, so the EKF linearizes these functions around the current estimate using Jacobian matrices.



## Project Structure
- slam/
    --- ekf_slam.py # Extended Kalman Filter SLAM
    --- pf_slam.py # Particle Filter alternative
- sensors/
    --- lidar_model.py # measurement and motion models
- data/
    --- sample_bags/ # pre-recorded LiDAR data
    --- loader.py # parse bag files
- viz/
    --- map_viz.py # map + trajectory visualization
- tests/
    --- test_ekf.py # unit tests for filter steps
- requirements.txt
- README.md

## How To Run
TODO

## References
- Probabilistic Robotics by Thrun, Burgard & Fox
- UTIAS Multi-Robot Cooperative Localization Dataset (https://asrl.utias.utoronto.ca/datasets/mrclam/index.html)

## Limitations
- The Data is recorded in indoor environment, so it may not work well outdoors.