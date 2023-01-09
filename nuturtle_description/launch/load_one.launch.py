"""
Starts all the nodes to visualize a turtlebot3 in rviz
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution, \
                                 LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.actions import Shutdown


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="use_jsp",
                              default_value="true",
                              choices=['true', 'false'],
                              description="Choose if joint_state_publisher needs to be launched."),

        DeclareLaunchArgument(name="use_rviz",
                              default_value="true",
                              choices=['true', 'false'],
                              description="Choose if RVIZ needs to be launched."),

        DeclareLaunchArgument(name="color",
                              default_value="purple",
                              choices=['red', 'green', 'blue', 'purple', ''],
                              description="Sets color of the turtlebot3 in the urdf."),

        SetLaunchConfiguration(name="rviz_color",
                               value=[FindPackageShare("nuturtle_description"),
                                      TextSubstitution(text="/config/basic_"),
                                      LaunchConfiguration("color"),
                                      TextSubstitution(text=".rviz")]
                               ),

        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             namespace=LaunchConfiguration("color"),
             condition=LaunchConfigurationEquals("use_jsp", "true")
             ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration("color"),
            parameters=[
                {
                 "frame_prefix":
                    PathJoinSubstitution([(LaunchConfiguration('color')), '']),
                 "robot_description":
                    Command([ExecutableInPackage("xacro", "xacro"), " ",
                            PathJoinSubstitution(
                            [FindPackageShare("nuturtle_description"),
                             "urdf/turtlebot3_burger.urdf.xacro"]),
                            " color:=",
                            LaunchConfiguration('color')
                                            ])},
                ]),

        Node(
            package="rviz2",
            executable="rviz2",
            name='rviz2',
            namespace=LaunchConfiguration("color"),
            arguments=["-d",
                       PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                             LaunchConfiguration("rviz_color")])],
            condition=LaunchConfigurationEquals("use_rviz", "true"),
            on_exit=Shutdown()
            )
        ])
