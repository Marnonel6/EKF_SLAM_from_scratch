#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/rigid2d.hpp"


namespace turtlelib
{
    std::ostream &operator<<(std::ostream &os, const Vector2D &v)
    {
        return os << "[" << v.x << " " << v.y << "]";
    }

    std::istream &operator>>(std::istream &is, Vector2D &v)
    {
        const auto  c = is.peek(); // examine the next character without extracting it

        if (c == '[')
        {
            is.get(); // remove the '[' character from the stream
            is >> v.x;
            is >> v.y;
            is.get(); // remove the ']' character from the stream
        }
        else
        {
            is >> v.x >> v.y;
        }

        is.ignore(100,'\n');
        return is;
    }

    std::ostream &operator<<(std::ostream &os, const Twist2D &t)
    {
        return os << "[" << t.w << " " << t.x << " " << t.y << "]";
    }

    std::istream &operator>>(std::istream &is, Twist2D &t)
    {
        const auto c = is.peek(); // examine the next character without extracting it

        if (c == '[')
        {
            is.get(); // remove the '[' character from the stream
            is >> t.w;
            is >> t.x;
            is >> t.y;
            is.get(); // remove the ']' character from the stream
        }
        else
        {
            is >> t.w >> t.x >> t.y;
        }
        is.ignore(100,'\n');

        return is;
    }

    Transform2D::Transform2D() : tran{0.0, 0.0}, rot(0.0){}

    Transform2D::Transform2D(Vector2D trans) : tran(trans), rot(0.0){}

    Transform2D::Transform2D(double radians) : tran{0.0, 0.0}, rot(radians){}

    Transform2D::Transform2D(Vector2D trans, double radians) : tran(trans), rot(radians){}

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        return {cos(rot)*v.x - sin(rot)*v.y + tran.x, sin(rot)*v.x + cos(rot)*v.y + tran.y};
    }

    Transform2D Transform2D::inv() const
    {
        return {{-tran.x*cos(rot)-tran.y*sin(rot), -tran.y*cos(rot)+tran.x*sin(rot)}, -rot};
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        tran.x = cos(rot)*rhs.tran.x - sin(rot)*rhs.tran.y + tran.x;
        tran.y = sin(rot)*rhs.tran.x + cos(rot)*rhs.tran.y + tran.y;
        rot = rot+rhs.rot;
        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        return tran;
    }

    double Transform2D::rotation() const
    {
        return rot;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        return os << "deg: " << rad2deg(tf.rot) << " " << "x: " << tf.tran.x << " " << "y: " << tf.tran.y;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        std::string str, str2, str3;
        double rot = 0.0;
        Vector2D tran{0.0,0.0};

        const auto c = is.peek(); // examine the next character without extracting it

        if (c == 'd')
        {
            // remove the 'deg: ' from the stream
            is >> str; // deg:
            is >> rot;
            // remove 'x: ' from the stream
            is >> str2; // x:
            is >> tran.x;
            // remove 'y: ' from the stream
            is>> str3; // y:
            is >> tran.y;
        }
        else
        {
             is >> rot >> tran.x >> tran.y; // Extract values from is buffer
        }
        is.ignore(100,'\n');
        // Change deg input to radians for calculations
        rot = deg2rad(rot);
        // Use constructer with values extracted from the is stream
        tf = Transform2D{tran, rot}; // Use reference to Transform2D objct tf to save input values

        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs*=rhs;
    }

    Twist2D Transform2D::operator()(Twist2D t) const
    {
        return {t.w, t.w*tran.y + t.x*cos(rot) - t.y*sin(rot), -t.w*tran.x + t.x*sin(rot) + t.y*cos(rot)};
    }

    Vector2D normalize(Vector2D v)
    {
        const auto length = sqrt(v.x * v.x + v.y * v.y);
        return {v.x/length, v.y/length};
    }

    double normalize_angle(double rad)
    {
        double rad_wrap = fmod(rad, 2.0*PI); // Angle wrapping - Modulas operand for floats
        if (rad_wrap > PI)
        {
            rad_wrap = -PI + (rad_wrap - PI); // -Pi side / CCW rotation
        }
        else if (rad_wrap <= -PI)
        {
            rad_wrap = PI - (rad_wrap + PI); // Pi side / CW rotation
        }
        return rad_wrap;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs)
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
    {
        return lhs+=rhs;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
    {
        return lhs-=rhs;
    }

    Vector2D & Vector2D::operator*=(const double & rhs)
    {
        x *= rhs;
        y *= rhs;
        return *this;
    }

    Vector2D operator*(Vector2D lhs, const double & rhs)
    {
        return lhs*=rhs;
    }

    Vector2D operator*(const double & lhs, Vector2D rhs)
    {
        return rhs*=lhs;
    }

    double dot(Vector2D v1, Vector2D v2)
    {
        return (double)(v1.x*v2.x + v1.y*v2.y);
    }

    double magnitude(Vector2D v)
    {
        return (double)std::sqrt(v.x*v.x + v.y*v.y);
    }

    double angle(Vector2D v1, Vector2D v2)
    {
        return atan2(v1.x*v2.y-v1.y*v2.x, v1.x*v2.x+v1.y*v2.y); // -pi, pi
    }

    Transform2D integrate_twist(Twist2D t)
    {
        if (t.w == 0)
        {
            double x = t.x;
            double y = t.y;
            Transform2D Tbb_prime(Vector2D{x,y}); // Pure Translation
            return Tbb_prime;
        }
        else
        {
            double x = t.y/t.w;
            double y = -t.x/t.w;
            Transform2D Tsb(Vector2D{x,y}); // Translation & Rotation or Pure Rotation
            Transform2D Tss_prime(t.w); // Pure rotation
            Transform2D Tb_prime_s_prime;
            Transform2D Ts_prime_b_prime;
            Transform2D Tbs;
            Transform2D Tbb_prime;
            Tbs = Tsb.inv();
            Tb_prime_s_prime = Tbs;
            Ts_prime_b_prime = Tb_prime_s_prime.inv();
            Tbb_prime = Tbs*Tss_prime*Ts_prime_b_prime;
            return Tbb_prime;
        }
    }
}
