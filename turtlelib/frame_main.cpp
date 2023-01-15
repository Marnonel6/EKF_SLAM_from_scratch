// #include <cstdio>
// #include <cmath>
#include <iostream>
#include "rigid2d.hpp"

using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::normalize;


int main()
{
    Transform2D Tab, Tba, Tbc, Tcb, Tac, Tca;
    Vector2D va, vb, vc;
    Twist2D Va, Vb, Vc;

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> Tbc;
    Tba = Tab.inv();
    Tcb = Tbc.inv();
    Tac = Tab*Tbc;
    Tca = Tac.inv();
    std::cout << "T_{a, b}: " << Tab << std::endl;
    std::cout << "T_{b, a}: " << Tba << std::endl;
    std::cout << "T_{b, c}: " << Tbc << std::endl;
    std::cout << "T_{c, b}: " << Tcb << std::endl;
    std::cout << "T_{a, c}: " << Tac << std::endl;
    std::cout << "T_{c, a}: " << Tca << std::endl;

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> vb;
    va = Tab(vb);
    vc = Tcb(vb);
    std::cout << "v_bhat " << normalize(vb) << std::endl;
    std::cout << "v_a " << va << std::endl;
    std::cout << "v_b " << vb << std::endl;
    std::cout << "v_c " << vc << std::endl;

    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> Vb;
    Va = Tab(Vb);
    Vc = Tcb(Vb);
    std::cout << "V_a " << Va << std::endl;
    std::cout << "V_b " << Vb << std::endl;
    std::cout << "V_c " << Vc << std::endl;

    return 0;
}
