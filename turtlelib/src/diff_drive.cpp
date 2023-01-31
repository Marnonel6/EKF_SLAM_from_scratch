#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive() : wheel_radius(0.033), wheel_track(0.16), q{0.0, 0.0, 0.0},
                             wheel_position{0.0, 0.0}{}

    /// \brief set wheel specifications and start at origin
    /// \param radius - radius of wheels
    /// \param track - distance between the wheels
    explicit DiffDrive::DiffDrive(double radius, double track) : wheel_radius(radius),
                                                                 wheel_track(track),
                                                                 q{0.0, 0.0, 0.0},
                                                                 wheel_position{0.0, 0.0}{}

    /// \brief set start configuration and wheel specifications
    /// \param radius - radius of wheels
    /// \param track - distance between the wheels
    /// \param robot_config - robot configuration
    explicit DiffDrive::DiffDrive(double radius, double track, Robot_configuration robot_config) :
                                    wheel_radius(radius),
                                    wheel_track(track),
                                    q{robot_config},
                                    wheel_position{0.0, 0.0}{}

}