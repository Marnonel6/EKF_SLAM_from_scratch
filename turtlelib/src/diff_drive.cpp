#include <iostream>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive() : wheel_radius(turtlebot3_wheel_radius),
                             wheel_track(turtlebot3_wheel_track), q{0.0, 0.0, 0.0},
                             wheel_position{0.0, 0.0}{}

    DiffDrive::DiffDrive(double radius, double track) : wheel_radius(radius),
                                                                 wheel_track(track),
                                                                 q{0.0, 0.0, 0.0},
                                                                 wheel_position{0.0, 0.0}{}

    DiffDrive::DiffDrive(double radius, double track, Robot_configuration robot_config) :
                                    wheel_radius(radius),
                                    wheel_track(track),
                                    q{robot_config},
                                    wheel_position{0.0, 0.0}{}

    WheelVelocities DiffDrive::InverseKinematics(Twist2D twist)
    {
        WheelVelocities wheel_vel;
        if (twist.y != 0.0)
        {
            throw std::logic_error("Twist cannot be accomplished without the wheels slipping!");
        }
        else
        {
            wheel_vel.left = (1/wheel_radius)*(-(wheel_track/2)*twist.w + twist.x);
            wheel_vel.right = (1/wheel_radius)*((wheel_track/2)*twist.w + twist.x);
        }
        return wheel_vel;
    }

}