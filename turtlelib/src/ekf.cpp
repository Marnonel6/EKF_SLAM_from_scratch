#include <iostream>
#include <armadillo>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"

namespace turtlelib
{
    EKF::EKF() : zai{m+2*n,arma::fill::zeros}, covariance{m+2*n,m+2*n,arma::fill::zeros}
    {
        initialize_covariance();
    }

    EKF::EKF(Robot_configuration robot_config): zai{m+2*n,arma::fill::zeros}, covariance{m+2*n,m+2*n,arma::fill::zeros}
    {
        initialize_covariance();
        initialize_robot_state(robot_config);
    }

    void EKF::initialize_covariance()
    {
        arma::mat cov_state, cov_obstacles, cov_zero1, cov_zero2; 
        cov_state = arma::mat {m,m,arma::fill::zeros};
        cov_obstacles =  arma::mat {2*n,2*n,arma::fill::eye}*1000000.0;
        cov_zero1 = arma::mat {m,2*n,arma::fill::zeros};
        cov_zero2 = arma::mat {2*n,m,arma::fill::zeros};
        covariance = arma::join_rows(arma::join_cols(cov_state,cov_zero2), arma::join_cols(cov_zero1,cov_obstacles));
    }

    void EKF::initialize_robot_state(Robot_configuration robot_config)
    {
        zai(0,0) = robot_config.theta;
        zai(1,0) = robot_config.x;
        zai(2,0) = robot_config.y;
    }

}
