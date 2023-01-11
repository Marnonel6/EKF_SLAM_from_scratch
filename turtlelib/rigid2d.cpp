#include <cstdio>
#include <cmath>
#include <iostream>
#include"rigid2d.hpp" // contains forward definitions for iostream objects

using turtlelib::Vector2D;

int main() {
    // Your code here

    Vector2D vec1;
    std::cin >> vec1;
    std::cout << vec1 << std::endl;

    printf("Done!\n");

    return 0;
}

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        return os << "[" << v.x << " " << v.y << "]";
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        char c = is.peek(); // brief examine the next character without extracting it
        if (c == '[') {
            is.get(); // remove the '[' character from the stream
            is >> v.x;
            is >> v.y;
            is.get(); // remove the ']' character from the stream
        } else {
            is >> v.x >> v.y;
        }
        return is;
    }
}
