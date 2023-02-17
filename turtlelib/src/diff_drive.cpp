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

    Twist2D DiffDrive::Twist(Wheel new_wheel_positions)
    {
        wheel_position.left = wheel_position.left + new_wheel_positions.left;
        wheel_position.right = wheel_position.right + new_wheel_positions.right;

        // This is change in wheel position not new_wheel_position. It will be updated.
        WheelVelocities wheel_vel;
        wheel_vel.left = new_wheel_positions.left; // - wheel_position.left;
        wheel_vel.right = new_wheel_positions.right; // - wheel_position.right;

        Twist2D twist_bbp;
        twist_bbp.w = (wheel_radius/2)*((-wheel_vel.left/(wheel_track/2)) + 
                      wheel_vel.right/(wheel_track/2));
        twist_bbp.x = (wheel_radius/2)*(wheel_vel.left + wheel_vel.right);
        twist_bbp.y = 0;
        return twist_bbp;
    }

    void DiffDrive::ForwardKinematics(Wheel new_wheel_positions)
    {
        // Equation 1 START //
        Twist2D twist_bbp = DiffDrive::Twist(new_wheel_positions);
        // Equation 1 End //

        // Equation 2 START //
        Transform2D Tb_bprime; // Transformation matrix between current and end position
        Tb_bprime = integrate_twist(twist_bbp);
        // Equation 2 END //

        // Equation 3 START //
        Transform2D Twb(Vector2D{q.x, q.y}, q.theta); // Current position to world
        // Equation 3 END //

        // Equation 4 START //
        Transform2D Tw_bprime; // End position to world
        Tw_bprime = Twb*Tb_bprime;
        // Equation 4 END //

        // Equation 5 START //
        // New configuration
        q.x = Tw_bprime.translation().x;
        q.y = Tw_bprime.translation().y;
        q.theta = normalize_angle(Tw_bprime.rotation());
        // Equation 5 END //
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
            // Equation 1 START //
            wheel_vel.left = (1/wheel_radius)*(-(wheel_track/2)*twist.w + twist.x);
            wheel_vel.right = (1/wheel_radius)*((wheel_track/2)*twist.w + twist.x);
            // Equation 1 END //
        }
        return wheel_vel;
    }

    Robot_configuration DiffDrive::configuration() const
    {
        return q;
    }

    void DiffDrive::set_configuration(Robot_configuration q_new)
    {
        q.x = q_new.x;
        q.y = q_new.y;
        q.theta = q_new.theta;
    }

}