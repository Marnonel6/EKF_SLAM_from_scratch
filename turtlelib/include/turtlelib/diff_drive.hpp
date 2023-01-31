#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics of a differential drive robot.

#include<iosfwd>
#include<cmath>

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double turtlebot3_wheel_radius=3.14159265358979323846;
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double turtlebot3_wheel_radius=3.14159265358979323846;

    /// \brief Position of both wheels
    struct Wheel_position
    {
        /// \brief left rotational wheel position in radians
        double left = 0.0;

        /// \brief right rotational wheel position in radians
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
        Wheel_position wheel_position;

    public:
        /// \brief start at origin and default to turtlebo3 burger specifications
        DiffDrive();

        /// \brief set wheel specifications and start at origin
        /// \param radius - radius of wheels
        /// \param track - distance between the wheels
        explicit DiffDrive(double radius, double track);

        /// \brief set start configuration and wheel specifications
        /// \param radius - radius of wheels
        /// \param track - distance between the wheels
        /// \param robot_config - robot configuration
        explicit DiffDrive(double radius, double track, Robot_configuration robot_config);
    };

}

#endif
