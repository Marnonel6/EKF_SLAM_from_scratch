#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/rigid2d.hpp"

using turtlelib::Transform2D;


namespace turtlelib
{
    std::ostream &operator<<(std::ostream &os, const Vector2D &v)
    {
        return os << "[" << v.x << " " << v.y << "]";
    }

    std::istream &operator>>(std::istream &is, Vector2D &v)
    {
        char c = is.peek(); // examine the next character without extracting it

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
        char c = is.peek(); // examine the next character without extracting it

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
        return Vector2D{cos(rot)*v.x - sin(rot)*v.y + tran.x, sin(rot)*v.x + cos(rot)*v.y + tran.y};
    }

    Transform2D Transform2D::inv() const
    {
        Transform2D trans2D;
        trans2D.tran.x = -tran.x*cos(rot)-tran.y*sin(rot);
        trans2D.tran.y = -tran.y*cos(rot)+tran.x*sin(rot);
        trans2D.rot = -rot;
        return trans2D;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        tran.x = cos(rot)*rhs.tran.x - sin(rot)*rhs.tran.y + tran.x;
        tran.y = sin(rot)*rhs.tran.x + cos(rot)*rhs.tran.y + tran.y;
        rot = fmod(rot+rhs.rot, 2*PI); // ANGLE WRAPPING??? MATT - Modulas operand
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

        char c = is.peek(); // examine the next character without extracting it

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
        // Chaneg deg input to radians for calculations
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
        Twist2D newTwist;
        newTwist.w = t.w;
        newTwist.x = t.w*tran.y + t.x*cos(rot) - t.y*sin(rot);
        newTwist.y = -t.w*tran.x + t.x*sin(rot) + t.y*cos(rot);
        return newTwist;
    }

    Vector2D normalize(Vector2D v)
    {
        double length = sqrt(v.x * v.x + v.y * v.y);
        return Vector2D{v.x/length, v.y/length};
    }

}
