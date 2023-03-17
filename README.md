# ME495 Sensing, Navigation and Machine Learning For Robotics
* Marthinus Nel
* Winter 2023
# Package List
This repository consists of several ROS2/C++ packages
- [nuturtle_description](https://github.com/ME495-Navigation/nuturtle-Marnonel6/tree/main/nuturtle_description) - Displays multiple Turtlebot3's in rviz.
- [turtlelib](https://github.com/ME495-Navigation/nuturtle-Marnonel6/tree/main/turtlelib) - A library for handling transformations in SE(2) and other turtlebot-related math.
- [nusim](https://github.com/ME495-Navigation/nuturtle-Marnonel6/blob/main/nusim)  - Simulation and visualization tool for the turtlebot3 robots in rviz.
- [nuturtle_control](https://github.com/ME495-Navigation/nuturtle-Marnonel6/tree/main/nuturtle_control) - This package provides functionalities for controlling the movement of a turtle robot and calculating its odometry.
- [nuslam](https://github.com/ME495-Navigation/nuturtle-Marnonel6/blob/main/nuslam/README.md) - This is a package that implements and runs an Extended Kalman Filter SLAM algorithm in a simulated/real world environment. Additionally it preforms lidar points clustering, circle fitting and unknown data association.


# Example Video

### Robot color:
* Red robot: Ground truth
* Blue robot: Odometry
* Green robot: EKF SLAM estimate

### Obstacle color:
* Red obstacle: Ground truth
* Green obstacle: Clustering algorithm -> Circle fitting algorithm -> Unknown Data Association

[Screencast from 03-16-2023 08:02:09 PM.webm](https://user-images.githubusercontent.com/60977336/225795689-4396b79f-5ccc-493c-8a90-72d061bc1e6e.webm)

## Final pose error between the actual robot position and odometry
- x: 0.98891 - 0.032898 = 0.956 [m]
- y: 0.1497 - 0.036449 = 0.113 [m]
- theta: 4.296 - 5.216 = -0.92 [Deg]

## Final pose error between the actual robot position and the SLAM estimate
- x: 0.03968 - 0.032898 = 0.007 [m]
- y: 0.046666 - 0.036449 = 0.01 [m]
- theta: 4.736 - 5.216 = -0.48 [Deg]

# Example

![Screenshot from 2023-03-01 19-48-34](https://user-images.githubusercontent.com/60977336/222309949-d921b76a-55d2-4852-8559-c2b7599159ab.png)
