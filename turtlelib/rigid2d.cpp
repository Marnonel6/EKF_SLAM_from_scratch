#include <cmath>
#include"rigid2d.hpp" // contains forward definitions for iostream objects

using namespace turtlelib;

constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
{
    return std::abs(d1 - d2) < epsilon;
}

constexpr double deg2rad(double deg)
{
    return (deg * PI) / 180;
}

constexpr double rad2deg(double rad)
{
    return (rad * 180) / PI;
}