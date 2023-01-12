#include <cstdio>
#include <cmath>
#include <iostream>
#include "rigid2d.hpp" // contains forward definitions for iostream objects

using turtlelib::Transform2D;

int main()
{
    // // When both is input with [] then the twist is printed out as [0 0 0] Why?
    // turtlelib::Vector2D vec1;
    // std::cin >> vec1;
    // std::cout << vec1 << std::endl;
    //
    // std::cin.ignore(10000,'\n');
    //
    // turtlelib::Twist2D vec2;
    // std::cin >> vec2;
    // std::cout << vec2 << std::endl;

    printf("Done!\n");

    return 0;
}

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
        // std::cin.ignore(10000,'\n'); // TODO: How would we ignore two of these 
        return is;
    }

    Transform2D::Transform2D() : tran{0.0, 0.0}, rot(0.0){}

    Transform2D::Transform2D(Vector2D trans) : tran(trans), rot(0.0){}

    Transform2D::Transform2D(double radians) : tran{0.0, 0.0}, rot(radians){}

    Transform2D::Transform2D(Vector2D trans, double radians) : tran(trans), rot(radians){}

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        Vector2D vec;
        vec.x = cos(rot)*v.x - sin(rot)*v.y + tran.x;
        vec.y = sin(rot)*v.x + cos(rot)*v.y + tran.y;
        return vec;
        // TODO: Which one is better
        // return Vector2D{cos(rot)*v.x - sin(rot)*v.y + tran.x, sin(rot)*v.x + cos(rot)*v.y + tran.y};
    }

    Transform2D Transform2D::inv() const
    {
        Transform2D trans2D;
        trans2D.tran.x = -tran.x*cos(rot)-tran.y*sin(rot);
        trans2D.tran.y = -tran.y*cos(rot)+tran.x*sin(rot);
        trans2D.rot = -rot;
        return trans2D;
    }

    // Transform2D & Transform2D::operator*=(const Transform2D & rhs) : {}
}
