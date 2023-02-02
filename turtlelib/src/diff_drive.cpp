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

    Robot_configuration DiffDrive::ForwardKinematics(Wheel new_wheel_positions)
    {
        Transform2D Twb(Vector2D{q.x, q.y}, q.theta); // Current position to world

        WheelVelocities wheel_vel;
        wheel_vel.left = new_wheel_positions.left - wheel_position.left;
        wheel_vel.right = new_wheel_positions.right - wheel_position.right;

        Twist2D twist_bbp;
        twist_bbp.w = (wheel_radius/2)*((-wheel_vel.left/(wheel_track/2)) + 
                      wheel_vel.right/(wheel_track/2));
        twist_bbp.x = (wheel_radius/2)*(wheel_vel.left + wheel_vel.right);
        twist_bbp.y = 0;
        Transform2D Tb_bprime; // Transformation matrix between current and end position
        Tb_bprime = integrate_twist(twist_bbp);

        Transform2D Tw_bprime; // End position to world
        Tw_bprime = Twb*Tb_bprime;

        Robot_configuration q_new; // New configuration
        q_new.x = Tw_bprime.translation().x;
        q_new.y = Tw_bprime.translation().y;
        q_new.theta = Tw_bprime.rotation();

        // Angle normalize ????

        return q_new;
    }

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