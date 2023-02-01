#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive() : wheel_radius(turtlebot3_wheel_radius),
                             wheel_track(turtlebot3_wheel_track), q{0.0, 0.0, 0.0},
                             wheel_position{0.0, 0.0}{}

    explicit DiffDrive::DiffDrive(double radius, double track) : wheel_radius(radius),
                                                                 wheel_track(track),
                                                                 q{0.0, 0.0, 0.0},
                                                                 wheel_position{0.0, 0.0}{}

    explicit DiffDrive::DiffDrive(double radius, double track, Robot_configuration robot_config) :
                                    wheel_radius(radius),
                                    wheel_track(track),
                                    q{robot_config},
                                    wheel_position{0.0, 0.0}{}

}