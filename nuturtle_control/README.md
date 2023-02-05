ros2 launch nuturtle_control start_robot.launch cmd_src:=teleop body_id:=base_footprint left_wheel_joint:=left_wheel right_wheel_joint:=right_wheel
ros2 service call /control nuturtle_control/srv/Control "{velocity: 1.0, radius: 1.0}"
ros2 service call /reverse std_srvs/srv/Empty "{}"
ros2 service call /stop std_srvs/srv/Empty "{}"
