#include <iostream>
#include <armadillo>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"

namespace turtlelib
{
    EKFSlam::EKFSlam() : zai{m+2*n,arma::fill::zeros}, covariance{m+2*n,m+2*n,arma::fill::zeros}
    {
        initialize_covariance();
    }

    EKFSlam::EKFSlam(Robot_configuration robot_config): zai{m+2*n,arma::fill::zeros}, covariance{m+2*n,m+2*n,arma::fill::zeros}
    {
        initialize_covariance();
        initialize_robot_state(robot_config);
    }

    void EKFSlam::initialize_covariance()
    {
        arma::mat cov_state, cov_obstacles, cov_zero1, cov_zero2; 
        cov_state = arma::mat {m,m,arma::fill::zeros};
        cov_obstacles =  arma::mat {2*n,2*n,arma::fill::eye}*1000000.0;
        cov_zero1 = arma::mat {m,2*n,arma::fill::zeros};
        cov_zero2 = arma::mat {2*n,m,arma::fill::zeros};
        covariance = arma::join_rows(arma::join_cols(cov_state,cov_zero2), arma::join_cols(cov_zero1,cov_obstacles));
    }

    void EKFSlam::initialize_robot_state(Robot_configuration robot_config)
    {
        zai(0,0) = robot_config.theta;
        zai(1,0) = robot_config.x;
        zai(2,0) = robot_config.y;
    }

    void EKFSlam::EKFSlam_Predict(Twist2D twist)
    {
        // Calculate commad twist
        ut(0,0) = normalize_angle(twist.w - prev_twist.w);
        ut(1,0) = twist.x - prev_twist.x;
        ut(2,0) = 0.0;
        // Set previous twist
        prev_twist = twist;

        // CALCULATE NEW CURRENT ESTIMATE STATE -> Zai_hat
        arma::colvec delta_state{m+2*n,arma::fill::zeros};
        if (almost_equal(ut(0,0), 0.0)) // Zero rotational velocity
        {
            delta_state(1,0) = ut(1,0)*cos(zai(1,0));
            delta_state(2,0) = ut(1,0)*sin(zai(1,0));

            // arma::colvec noise{m+2*n,arma::fill::zeros};
            // noise(0,0) = 
            // noise(1,0) = 
            // noise(2,0) = 

            zai_estimate = zai + delta_state; //+ noise;
        }
        else // Non-zero rotational velocity
        {
            delta_state(0,0) = ut(0,0);
            delta_state(1,0) = -(ut(1,0)/ut(0,0))*sin(zai(1,0)) + (ut(1,0)/ut(0,0))*sin(zai(1,0)+ut(0,0));
            delta_state(2,0) = (ut(1,0)/ut(0,0))*cos(zai(1,0)) + (ut(1,0)/ut(0,0))*cos(zai(1,0)+ut(0,0));

            // arma::colvec noise{m+2*n,arma::fill::zeros};
            // noise(0,0) = 
            // noise(1,0) = 
            // noise(2,0) = 

            zai_estimate = zai + delta_state; //+ noise;
        }




        // CALCULATE At matrix
        arma::mat zero_3_2n{m,2*n,arma::fill::zeros};
        arma::mat zero_2n_2n{m,2*n,arma::fill::zeros};
        arma::mat temp(m,m,arma::fill::zeros);
        if (almost_equal(ut(0,0), 0.0)) // Zero rotational velocity
        {
            temp(1,0) = -(ut(1,0))*sin(zai(1,0));
            temp(2,0) = ut(1,0)*cos(zai(1,0));
            At = I + arma::join_rows(arma::join_cols(temp,zero_3_2n.t()),arma::join_cols(zero_3_2n,zero_2n_2n));
        }
        else // Non-zero rotational velocity
        {
            temp(1,0) = -(ut(1,0)/ut(0,0))*cos(zai(1,0)) + (ut(1,0)/ut(0,0))*cos(zai(1,0)+ut(0,0));
            temp(2,0) = -(ut(1,0)/ut(0,0))*sin(zai(1,0)) + (ut(1,0)/ut(0,0))*sin(zai(1,0)+ut(0,0));
            At = I + arma::join_rows(arma::join_cols(temp,zero_3_2n.t()),arma::join_cols(zero_3_2n,zero_2n_2n));
        }

        Q_bar = arma::join_rows(arma::join_cols(Q,zero_3_2n.t()),arma::join_cols(zero_3_2n,zero_2n_2n));
        covariance_estimate = At*covariance*At.t() + Q_bar;
    }

}
