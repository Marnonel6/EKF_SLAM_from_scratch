# Turtle Control
* Author: Marthinus Nel
* Published: Winter 2023
# Package description
This package provides functionalities for controlling the movement of a turtle robot and calculating
its odometry. It consists of three nodes: circle node, turtle_control node, and odometry node. The
circle node is responsible for publishing twist messages to drive the turtle in a circle, the
turtle_control node subscribes to the twist messages and converts it into wheel commands. It also
subscribes to sensor data and converts it into joint states. The odometry node calculates the
odometry of the robot and publishes its location. The package provides services for reversing the
twist message, stopping the publishing, and setting the start position of the robot.

# Launchfile description
- `start_robot.launch.xml`:
    ### Launch arguments:
    * `cmd_src`: Specifies which node publishes to the cmd_vel topic.
        - circle - Run circle node
        - teleop - Run teleop_twist_keyboard
        - none - Other source publishes to cmd_vel
    * `robot`: Specifies if the simulation or physical robot is used.
        - nusim - Launch simulation launch file
        - localhost - Run numsr_turtlebot on physical robot
        - none - No robot
    * `use_rviz`: Launches rviz if true

    * The launch file launches a blue turtlebot3 for odometry representatin with the launch file
      `nuturtle_description/launch/load_one.launch.py`.
    * The node `turtle_control` and `odometry` is always launched.
    * A static broadcaster broadcasts the transform between `nusim/world` and `odom`.
    * Rviz is launched to display the robots and the environment.
    * `ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=<X> use_rviz:=<Y> robot:=<Z>`

# Simulation
     ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=circle use_rviz:=true robot:=nusim
    
[Screencast from 02-08-2023 09:24:08 PM.webm](https://user-images.githubusercontent.com/60977336/217710169-f161eccf-7cdb-4175-a96f-3732156d67a9.webm)

# Physical robot - Launch on turtlebot3
     ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=circle use_rviz:=false robot:=localhost
    

[Screencast from 02-08-2023 09:08:01 PM.webm](https://user-images.githubusercontent.com/60977336/217708150-05fbc153-caa0-4c85-a106-7f2be1badcbd.webm)

# Service's for circle node
- Circle control service:
    *        `ros2 service call /control nuturtle_control/srv/Control "{velocity: 0.2, radius: 0.5}"`
- Reverse service:
    *       `ros2 service call /reverse std_srvs/srv/Empty "{}"`
- Stop service:
    *       `ros2 service call /stop std_srvs/srv/Empty "{}"`

# Parameters
* ```frequency``` (int): Timer callback frequency [Hz]
* ```wheelradius``` (float): The radius of the wheels [m]
* ```track_width``` (float): The distance between the wheels [m]
* ```motor_cmd_max``` (float): Maximum motor command value in ticks velocity
* ```motor_cmd_per_rad_sec``` (float): Motor command to rad/s conversion factor
* ```encoder_ticks_per_rad``` (float): Encoder ticks to radians conversion factor
* ```collision_radius``` (float): Robot collision radius [m]
* ```body_id``` (std::string): The name of the body frame of the robot
* ```odom_id``` (std::string): The name of the odometry frame
* ```left_wheel_joint```: The name of the left wheel joint
* ```left_wheel_joint```: The name of the right wheel joint
* ```wheelradius``` (float): The radius of the wheels [m]
* ```track_width``` (float): The distance between the wheels [m]

# Odometry accuracy test:
* Start:
    - x = 0.1317 [m]
    - y = -0.0197 [m]
    - theta = 17.02 [deg]
* End:
    - x = 0.1866 [m]
    - y = -0.0356 [m]
    - theta = 3.04 [deg]
* Error:
    - x = 0.0549 [m]
    - y = 0.0159 [m]
    - theta = 13.98 [deg]
