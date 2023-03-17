# Nusim
* Author: Marthinus Nel
* Published: Winter 2023
# Package description
This is a package to implement and run an Extended Kalman Filter SLAM algorithm in a simulated/real
world environment. It uses a clustering, circle fitting and data association algorithm frim the custom
`ekf library`. The algorithm is designed to compute the pose of the robot and landmarks relative
to a map frame, and it publishes a transform from map to odom such that map to base_footprint
reflects the pose of the (green) robot in the map frame. The launchfile includes the necessary nodes
for running the algorithm and controlling the robot. The data association is assumed to be known,
meaning that the algorithm knows which measurement goes with which landmark. This task is part of a
larger project on robot mapping and localization.

# SLAM launchfile description
- `slam.launch.xml`:
    ### Launch arguments:
    * `cmd_src`: Specifies which node publishes to the cmd_vel topic.
        - circle - Run circle node
        - teleop - Run teleop_twist_keyboard
        - none - Other source publishes to cmd_vel
    * `robot`: Specifies if the simulation or physical robot is used.
        - nusim - Launch simulation launch file
        - localhost - Run numsr_turtlebot on physical robot
        - none - Displays SLAM obstacles, actual obstacles, walls and odometry (blue) robot
    * `use_rviz`: Launches rviz if true
    * The `slam` node is launched and the green robot that preforms EKF Slam with the ekf library.
    * Rviz is launched to display the robots and the environment.
    * `ros2 launch nuslam slam.launch.xml cmd_src:=<X> use_rviz:=<Y> robot:=<Z>`

# Landmark launchfile description
- `landmark_detect.launch.xml`:
    ### Launch arguments:
    * `robot`: Specifies if the simulation or physical robot is used.
        - nusim - Launch simulation launch file
        - localhost - Run numsr_turtlebot on physical robot
        - none - Displays SLAM obstacles, actual obstacles, walls and odometry (blue) robot
    * `use_landmarks`: Use output of landmarks (circle fitting) for SLAM.
        - true - Use output of landmarks (circle fitting) for SLAM.
        - false - Use fake sensor data.
    * The `slam.launch.xml` launchfile is launched and the green robot that preforms EKF Slam with the ekf library.
    * Rviz is launched to display the robots and the environment.
    * `ros2 launch nuslam landmark_detect.launch.xml use_landmarks:=<X>`

# Unknown data association launchfile description
- `unknown_data_assoc.launch.xml`:
    ### Launch arguments:
    * `robot`: Specifies if the simulation or physical robot is used.
        - nusim - Launch simulation launch file
        - localhost - Run numsr_turtlebot on physical robot
        - none - Displays SLAM obstacles, actual obstacles, walls and odometry (blue) robot
    * `use_landmarks`: Use output of landmarks (circle fitting) for SLAM.
        - true - Use output of landmarks (circle fitting) for SLAM.
        - false - Use fake sensor data.
    * The `landmark_detect.launch.xml` launchfile is launched and the green robot that preforms clustering, circle fitting, unknown data association and EKF Slam with the ekf library.
    * Rviz is launched to display the robots and the environment.
    * `ros2 launch nuslam unknown_data_assoc.launch.xml use_landmarks:=<X>`

# Parameters
* ```body_id``` (std::string): The name of the body frame of the robot
* ```odom_id``` (std::string): The name of the odometry frame
* ```wheel_left``` (std::string): The name of the left wheel joint
* ```wheel_right``` (std::string): The name of the right wheel joint
* ```wheelradius``` (double): The radius of the wheels [m]
* ```track_width``` (double): The distance between the wheels [m]
* ```obstacles.r``` (double): Radius of cylindrical obstacles [m]
* ```obstacles.h``` (double): Height of cylindrical obstacles [m]

# Example Video

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


