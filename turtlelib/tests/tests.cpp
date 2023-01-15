/*
Contributers:
    - Marno, Nel
    - Katie, Hughes
    - Megan, Sindelar
    - Ava, Zahedi
*/
#include <catch2/catch_test_macros.hpp>
#include <sstream>
#include "turtlelib/rigid2d.hpp"

using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::almost_equal;
using turtlelib::PI;


TEST_CASE("rotation()","[transform]") // Marno, Nel
{
    double test_rot = 60.0;
    Transform2D T_test{test_rot};
    REQUIRE(T_test.rotation() == test_rot);
}

TEST_CASE("translation()","[transform]") // Marno, Nel
{
    double test_x = 4.20;
    double test_y = 6.9;
    Transform2D T_test{{test_x,test_y}};
    REQUIRE(T_test.translation().x == test_x);
    REQUIRE(T_test.translation().y == test_y);
}

TEST_CASE("operator()(Vector2D v)","[transform]") // Marno, Nel
{
    double test_rot = PI/2.0;
    double test_x = 0.0;
    double test_y = 1.0;
    Transform2D T_ab{{test_x,test_y}, test_rot};
    Vector2D v_b{1, 1};
    Vector2D v_a = T_ab(v_b);
    REQUIRE(almost_equal(v_a.x, -1.0));
    REQUIRE(almost_equal(v_a.y, 2.0));
}

TEST_CASE("operator()(Twist2D t)","[transform]") // Marno, Nel
{
    double test_rot = PI/2.0;
    double test_x = 0.0;
    double test_y = 1.0;
    Transform2D T_ab{{test_x,test_y}, test_rot};
    Twist2D V_b{1, 1, 1};
    Twist2D V_a = T_ab(V_b);
    REQUIRE(almost_equal(V_a.w, 1.0));
    REQUIRE(almost_equal(V_a.x, 0.0));
    REQUIRE(almost_equal(V_a.y, 1.0));
}

TEST_CASE("inverse - inv()", "[transform]")  // Adapted from Katie, Hughes
{
   double test_rot = PI/2.0;
   double test_x = 0.0;
   double test_y = 1.0;
   Transform2D Ttest{{test_x,test_y}, test_rot};
   Transform2D Ttest_inv = Ttest.inv();
   REQUIRE(Ttest.inv().rotation() == -test_rot);
   REQUIRE(almost_equal(Ttest_inv.translation().x, -1.0));
   REQUIRE(almost_equal(Ttest_inv.translation().y,  0.0));
}

TEST_CASE("stream insertion operator <<", "[transform]") // Adapted from Ava, Zahedi
{
   Vector2D vec;
   vec.x = 1.0;
   vec.y = 3.4;
   double phi = 0.0;
   Transform2D tf = Transform2D(vec, phi);
   std::string str = "deg: 0 x: 1 y: 3.4";
   std::stringstream sstr;
   sstr << tf;
   REQUIRE(sstr.str() == str);
}

TEST_CASE("operator *=", "[transform]") // Adapted from Megan, Sindelar
{
   Transform2D T_ab = {{1,2}, 0.0};
   Transform2D T_bc = {{3,4}, PI/2.0};
   Transform2D T_ac = T_ab*T_bc;
   REQUIRE(T_ac.translation().x == 4.0);
   REQUIRE(T_ac.translation().y == 6.0);
   REQUIRE(T_ac.rotation() == (PI/2));
}
