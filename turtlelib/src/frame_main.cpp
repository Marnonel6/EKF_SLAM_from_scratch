#include <iostream>
#include "turtlelib/rigid2d.hpp"

using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::normalize;


int main()
{
    Transform2D Tab, Tba, Tbc, Tcb, Tac, Tca;

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> Tbc;
    Tba = Tab.inv();
    Tcb = Tbc.inv();
    Tac = Tab*Tbc;
    Tca = Tac.inv();
    std::cout << "T_{a, b}: " << Tab << "\n";
    std::cout << "T_{b, a}: " << Tba << "\n";
    std::cout << "T_{b, c}: " << Tbc << "\n";
    std::cout << "T_{c, b}: " << Tcb << "\n";
    std::cout << "T_{a, c}: " << Tac << "\n";
    std::cout << "T_{c, a}: " << Tca << std::endl;

    Vector2D va, vb, vc;

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> vb;
    va = Tab(vb);
    vc = Tcb(vb);
    std::cout << "v_bhat " << normalize(vb) << "\n";
    std::cout << "v_a " << va << "\n";
    std::cout << "v_b " << vb << "\n";
    std::cout << "v_c " << vc << std::endl;

    Twist2D Va, Vb, Vc;

    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> Vb;
    Va = Tab(Vb);
    Vc = Tcb(Vb);
    std::cout << "V_a " << Va << "\n";
    std::cout << "V_b " << Vb << "\n";
    std::cout << "V_c " << Vc << std::endl;

    return 0;
}
