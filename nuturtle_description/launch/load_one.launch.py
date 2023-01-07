"""
Starts all the nodes to visualize a turtlebot3 in rviz
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.actions import Shutdown

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="use_jsp",
                              default_value="true",
                              choices=['true', 'false'],
                              description="true: use joint_state_publisher, false: no joint states \
                                           published"),
        DeclareLaunchArgument(name="use_rviz",
                              default_value="true",
                              choices=['true', 'false'],
                              description="true: start rviz, false: don't start rviz"),
        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             condition= LaunchConfigurationEquals("use_jsp", "true")
             ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description" :
                Command([ExecutableInPackage("xacro", "xacro"), " ",
                         PathJoinSubstitution(
                        [FindPackageShare("nuturtle_description"), 
                                          "urdf/turtlebot3_burger.urdf.xacro"])])}
                ]
                ),
        Node(
            package="rviz2",
            executable="rviz2",
            name='rviz2',
            arguments=["-d",
                       PathJoinSubstitution(
                      [FindPackageShare("nuturtle_description"), "config/basic_purple.rviz"])],
            condition=LaunchConfigurationEquals("use_rviz", "true"),
            on_exit = Shutdown()
            )
        ])