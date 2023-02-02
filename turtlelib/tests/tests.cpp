/*
Contributers:
    - Marno, Nel
    - Katie, Hughes
    - Megan, Sindelar
    - Ava, Zahedi
*/
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include "turtlelib/rigid2d.hpp"

using Catch::Matchers::WithinAbs;
using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::almost_equal;
using turtlelib::deg2rad;
using turtlelib::normalize_angle;
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
    REQUIRE_THAT(v_a.x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(v_a.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
}

TEST_CASE("operator()(Twist2D t)","[transform]") // Marno, Nel
{
    double test_rot = PI/2.0;
    double test_x = 0.0;
    double test_y = 1.0;
    Transform2D T_ab{{test_x,test_y}, test_rot};
    Twist2D V_b{1, 1, 1};
    Twist2D V_a = T_ab(V_b);
    REQUIRE_THAT(V_a.w, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(V_a.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(V_a.y, Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("inverse - inv()", "[transform]")  // Adapted from Katie, Hughes
{
    double test_rot = PI/2.0;
    double test_x = 0.0;
    double test_y = 1.0;
    Transform2D Ttest{{test_x,test_y}, test_rot};
    Transform2D Ttest_inv = Ttest.inv();
    REQUIRE(Ttest.inv().rotation() == -test_rot);
    REQUIRE_THAT(Ttest_inv.translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(Ttest_inv.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
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

TEST_CASE("stream extraction operator >>", "[transform]") // Adapted from Ava, Zahedi
{
    Transform2D tf = Transform2D();
    std::stringstream sstr;
    sstr << "deg: 90 x: 1 y: 3.4";
    sstr >> tf;
    REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(deg2rad(90), 1e-5));
    REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(3.4, 1e-5));
}

TEST_CASE("operator *=", "[transform]") // Adapted from Megan, Sindelar
{
    Vector2D trans_ab = {1.0, 2.0};
    double rotate_ab = 0.0;
    Transform2D T_ab_1 = {trans_ab, rotate_ab}; //T_ab's are all the same,
    Transform2D T_ab_2 = {trans_ab, rotate_ab}; //but, need different vars
    Transform2D T_ab_3 = {trans_ab, rotate_ab}; //b/c getting overwritten otherwise
    Vector2D trans_bc = {3.0, 4.0};
    double rotate_bc = PI/2.0;
    Transform2D T_bc = {trans_bc, rotate_bc};
    REQUIRE_THAT((T_ab_1*=T_bc).translation().x, Catch::Matchers::WithinAbs(4.0, 1e-5));
    REQUIRE_THAT((T_ab_2*=T_bc).translation().y, Catch::Matchers::WithinAbs(6.0, 1e-5));
    REQUIRE_THAT((T_ab_3*=T_bc).rotation(), Catch::Matchers::WithinAbs(PI/2.0, 1e-5));
}

TEST_CASE("normalize_angle()","[transform]") // Marno, Nel
{
    REQUIRE_THAT(normalize_angle(PI), Catch::Matchers::WithinAbs(PI, 1e-5));
    REQUIRE_THAT(normalize_angle(-PI), Catch::Matchers::WithinAbs(PI, 1e-5));
    REQUIRE_THAT(normalize_angle(0.0), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(normalize_angle(-PI/4.0), Catch::Matchers::WithinAbs(-PI/4.0, 1e-5));
    REQUIRE_THAT(normalize_angle(3.0*PI/2.0), Catch::Matchers::WithinAbs(-PI/2.0, 1e-5));
    REQUIRE_THAT(normalize_angle(-5.0*PI/2.0), Catch::Matchers::WithinAbs(-PI/2.0, 1e-5));
}

TEST_CASE("operator +", "[transform]") // Marno, Nel
{
    Vector2D v1 = {1.5, 2.0};
    Vector2D v2 = {2.0, -3.5};
    REQUIRE_THAT((v1+v2).x, Catch::Matchers::WithinAbs(3.5, 1e-5));
    REQUIRE_THAT((v1+v2).y, Catch::Matchers::WithinAbs(-1.5, 1e-5));
}

TEST_CASE("operator -", "[transform]") // Marno, Nel
{
    Vector2D v1 = {1.5, 2.0};
    Vector2D v2 = {2.0, -3.5};
    REQUIRE_THAT((v1-v2).x, Catch::Matchers::WithinAbs(-0.5, 1e-5));
    REQUIRE_THAT((v1-v2).y, Catch::Matchers::WithinAbs(5.5, 1e-5));
}

TEST_CASE("operator scalar*Vector2D", "[transform]") // Scalar first - Marno, Nel
{
    double scalar = 2.0;
    Vector2D v2 = {2.0, -3.5};
    REQUIRE_THAT((scalar*v2).x, Catch::Matchers::WithinAbs(4.0, 1e-5));
    REQUIRE_THAT((scalar*v2).y, Catch::Matchers::WithinAbs(-7.0, 1e-5));
}

TEST_CASE("operator Vector2D*scalar", "[transform]") // Scalar second - Marno, Nel
{
    double scalar = 2.0;
    Vector2D v2 = {2.0, -3.5};
    REQUIRE_THAT((v2*scalar).x, Catch::Matchers::WithinAbs(4.0, 1e-5));
    REQUIRE_THAT((v2*scalar).y, Catch::Matchers::WithinAbs(-7.0, 1e-5));
}

TEST_CASE("dot()", "[transform]") // Marno, Nel
{
    Vector2D v1 = {1.5, 2.0};
    Vector2D v2 = {2.0, -3.5};
    double dot_product = turtlelib::dot(v1, v2);
    REQUIRE_THAT(dot_product, Catch::Matchers::WithinAbs(-4.0, 1e-5));
}

TEST_CASE("magnitude()", "[transform]") // Marno, Nel
{
    Vector2D v1 = {2.0, -3.5};
    double mag = turtlelib::magnitude(v1);
    REQUIRE_THAT(mag, Catch::Matchers::WithinAbs(4.0311288, 1e-5));
}

TEST_CASE("angle()", "[transform]") // Marno, Nel
{
    Vector2D v1 = {1.5, 2.0};
    Vector2D v2 = {2.0, -3.5};
    Vector2D v3 = {1, 0};
    Vector2D v4 = {0, 1};
    double ang1 = turtlelib::angle(v1, v2);
    double ang2 = turtlelib::angle(v3, v4);
    REQUIRE_THAT(ang1, Catch::Matchers::WithinAbs(-1.9789454305450768778, 1e-5));
    REQUIRE_THAT(ang2, Catch::Matchers::WithinAbs(PI/2, 1e-5));
}

TEST_CASE("integrate_twist()", "[transform]") // Marno, Nel
{
    // Pure traslation
    Twist2D t1 = {0.0, -1.5, 1.0};
    Transform2D T1 = turtlelib::integrate_twist(t1);
    REQUIRE_THAT(T1.translation().x, Catch::Matchers::WithinAbs(-1.5, 1e-5));
    REQUIRE_THAT(T1.translation().y, Catch::Matchers::WithinAbs(1.0, 1e-5));
    // Pure rotation
    Twist2D t2 = {-28.0, 0.0, 0.0};
    Transform2D T2 = turtlelib::integrate_twist(t2);
    REQUIRE_THAT(T2.rotation(), Catch::Matchers::WithinAbs(-28.0, 1e-5));
    // Rotation and Traslation
    Twist2D t3 = {-1.24, -2.15, -2.92};
    Transform2D T3 = turtlelib::integrate_twist(t3);
    REQUIRE_THAT(T3.translation().x, Catch::Matchers::WithinAbs(-3.229863264722, 1e-5));
    REQUIRE_THAT(T3.translation().y, Catch::Matchers::WithinAbs(-1.05645265317421, 1e-5));
    REQUIRE_THAT(T3.rotation(), Catch::Matchers::WithinAbs(-1.24, 1e-5));
}

TEST_CASE("ForwardKinematics()", "[transform]") // Marno, Nel
{
    Vector2D v1 = {1.5, 2.0};
    Vector2D v2 = {2.0, -3.5};
    Vector2D v3 = {1, 0};
    Vector2D v4 = {0, 1};
    double ang1 = turtlelib::angle(v1, v2);
    double ang2 = turtlelib::angle(v3, v4);
    REQUIRE_THAT(ang1, Catch::Matchers::WithinAbs(-1.9789454305450768778, 1e-5));
    REQUIRE_THAT(ang2, Catch::Matchers::WithinAbs(PI/2, 1e-5));
}

TEST_CASE("InverseKinematics()", "[transform]") // Marno, Nel
{
    Vector2D v1 = {1.5, 2.0};
    Vector2D v2 = {2.0, -3.5};
    Vector2D v3 = {1, 0};
    Vector2D v4 = {0, 1};
    double ang1 = turtlelib::angle(v1, v2);
    double ang2 = turtlelib::angle(v3, v4);
    REQUIRE_THAT(ang1, Catch::Matchers::WithinAbs(-1.9789454305450768778, 1e-5));
    REQUIRE_THAT(ang2, Catch::Matchers::WithinAbs(PI/2, 1e-5));
}
