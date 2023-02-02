#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics of a differential drive robot.

#include<iosfwd>
#include<cmath>
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{
    /// \brief the wheel radius of the turtlebot3 burger
    constexpr double turtlebot3_wheel_radius=0.033;
    /// \brief the wheel track of the turtlebot3 burger
    constexpr double turtlebot3_wheel_track=0.16;

    /// \brief Position of both wheels
    struct Wheel
    {
        /// \brief left rotational wheel position in radians
        double left = 0.0;

        /// \brief right rotational wheel position in radians
        double right = 0.0;
    };

    /// \brief Velocity of both wheels
    struct WheelVelocities
    {
        /// \brief left wheel velocity
        double left = 0.0;

        /// \brief right wheel velocity
        double right = 0.0;
    };

    /// \brief Robot configuration
    struct Robot_configuration
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief the orientation angle
        double theta = 0.0;
    };

    /// \brief Kinematics of a differential drive robot.
    class DiffDrive
    {
    private:
        /// \brief radius of wheels
        double wheel_radius;
        /// \brief distance between the wheels
        double wheel_track;
        /// \brief robot configuration
        Robot_configuration q;
        /// \brief position of both wheels
        Wheel wheel_position;

    public:
        /// \brief start at origin and default to turtlebo3 burger specifications
        DiffDrive();

        /// \brief set wheel specifications and start at origin
        /// \param radius - radius of wheels
        /// \param track - distance between the wheels
        DiffDrive(double radius, double track);

        /// \brief set start configuration and wheel specifications
        /// \param radius - radius of wheels
        /// \param track - distance between the wheels
        /// \param robot_config - robot configuration
        DiffDrive(double radius, double track, Robot_configuration robot_config);

        /// \brief Calculates the forward kinematics from the new wheel positions
        /// \param new_wheel_positions - new wheel positions
        /// \return new robot configuration
        Robot_configuration ForwardKinematics(Wheel new_wheel_positions);

        /// \brief Calculates the inverse kinematics from a body twist
        /// \param twist - the body twist to preform inverse kinematics on
        /// \return wheel velocities required for the twist
        WheelVelocities InverseKinematics(Twist2D twist);
    };

}

#endif
