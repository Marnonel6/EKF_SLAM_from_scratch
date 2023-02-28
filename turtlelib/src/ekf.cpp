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
        zai(0) = robot_config.theta;
        zai(1) = robot_config.x;
        zai(2) = robot_config.y;
    }

    void EKFSlam::EKFSlam_Predict(Twist2D twist)
    {
        // Calculate commad twist
        ut(0) = normalize_angle(twist.w - prev_twist.w);
        ut(1) = twist.x - prev_twist.x;
        ut(2) = twist.y - prev_twist.y;
        // Set previous twist
        prev_twist = twist;

        // zai_estimate(0) += zai(0) + ut(0);
        // zai_estimate(1) += zai(1) + ut(1);
        // zai_estimate(2) += zai(2) + ut(2);

        zai_estimate(0) = zai(0) + ut(0);
        zai_estimate(1) = zai(1) + ut(1);
        zai_estimate(2) = zai(2) + ut(2);


        // // CALCULATE NEW CURRENT ESTIMATE STATE -> Zai_hat
        // arma::colvec delta_state{m+2*n,arma::fill::zeros};
        // if (almost_equal(ut(0), 0.0)) // Zero rotational velocity
        // {
        //     delta_state(1) = ut(1)*cos(zai(0));
        //     delta_state(2) = ut(1)*sin(zai(0));

        //     // arma::colvec noise{m+2*n,arma::fill::zeros};
        //     // noise(0,0) = 
        //     // noise(1,0) = 
        //     // noise(2,0) = 

        //     zai_estimate += zai + delta_state; //+ noise;
        // }
        // else // Non-zero rotational velocity
        // {
        //     delta_state(0) = ut(0);
        //     delta_state(1) = -((ut(1)/ut(0))*sin(zai(0))) + (ut(1)/ut(0))*sin(zai(0)+ut(0));
        //     delta_state(2) = (ut(1)/ut(0))*cos(zai(0)) - (ut(1)/ut(0))*cos(zai(0)+ut(0));

        //     // arma::colvec noise{m+2*n,arma::fill::zeros};
        //     // noise(0,0) = 
        //     // noise(1,0) = 
        //     // noise(2,0) = 

        //     zai_estimate += zai + delta_state; //+ noise;
        // }


        // CALCULATE At matrix
        arma::mat zero_3_2n{m,2*n,arma::fill::zeros};
        arma::mat zero_2n_2n{2*n,2*n,arma::fill::zeros};
        arma::mat temp(m,m,arma::fill::zeros);
        if (almost_equal(ut(0), 0.0)) // Zero rotational velocity
        {
            temp(1,0) = -(ut(1))*sin(zai(0));
            temp(2,0) = ut(1)*cos(zai(0));
            At = I + arma::join_rows(arma::join_cols(temp,zero_3_2n.t()),arma::join_cols(zero_3_2n,zero_2n_2n));
        }
        else // Non-zero rotational velocity
        {
            temp(1,0) = -(ut(1)/ut(0))*cos(zai(0)) + (ut(1)/ut(0))*cos(zai(0)+ut(0));
            temp(2,0) = -(ut(1)/ut(0))*sin(zai(0)) + (ut(1)/ut(0))*sin(zai(0)+ut(0));
            At = I + arma::join_rows(arma::join_cols(temp,zero_3_2n.t()),arma::join_cols(zero_3_2n,zero_2n_2n));
        }

        Q_bar = arma::join_rows(arma::join_cols(Q,zero_3_2n.t()),arma::join_cols(zero_3_2n,zero_2n_2n));
        covariance_estimate = At*covariance*At.t() + Q_bar;
    }

    void EKFSlam::EKFSlam_Correct(double x, double y, size_t j)
    {
        // Convert relative measurements to range bearing
        double r_j = std::sqrt(x*x + y*y);
        double phi_j = std::atan2(y,x); // Normalize ?? TODO ??

        // Check if landmark has been seen before
        // if (auto search = seen_landmarks.find(j); search != seen_landmarks.end())
        if (seen_landmarks.find(j) == seen_landmarks.end())
        {
            // Initialize the landmark estimate x and y coordinates in zai_estimate
            zai_estimate(m+2*j) = zai_estimate(1) + r_j*cos(phi_j + zai_estimate(0));
            zai_estimate(m+2*j+1) = zai_estimate(2) + r_j*sin(phi_j + zai_estimate(0));
            // Insert the new landmark index in the unordered_set
            seen_landmarks.insert(j);
        }

        // Actual measurements
        zj(0) = r_j;
        zj(1) = phi_j;

        // Estimate measurements
        Vector2D estimate_rel_dist_j;
        estimate_rel_dist_j.x = zai_estimate(m+2*j) - zai_estimate(1);
        estimate_rel_dist_j.y = zai_estimate(m+2*j+1) - zai_estimate(2);
        double d_j = estimate_rel_dist_j.x*estimate_rel_dist_j.x + estimate_rel_dist_j.y*estimate_rel_dist_j.y;
        double r_j_hat = std::sqrt(d_j);
        double phi_j_hat = normalize_angle(atan2(estimate_rel_dist_j.y, estimate_rel_dist_j.x) - zai_estimate(0));
        zj_hat(0) = r_j_hat; // TODO NO ADDED NOISE?? eq13
        zj_hat(1) = phi_j_hat;

        // Calculate H matrix
        arma::mat zeros_1j(2,2*j);
        arma::mat zeros_1nj(2,2*n - 2*(j+1));
        arma::mat temp1(2,3);
        arma::mat temp2(2,2);

        temp1(1,0) = -1;
        temp1(0,1) = -estimate_rel_dist_j.x/std::sqrt(d_j);
        temp1(1,1) = estimate_rel_dist_j.y/d_j;
        temp1(0,2) = -estimate_rel_dist_j.y/std::sqrt(d_j);
        temp1(1,2) = -estimate_rel_dist_j.x/d_j;

        temp2(0,0) = estimate_rel_dist_j.x/std::sqrt(d_j);
        temp2(1,0) = -estimate_rel_dist_j.y/d_j;
        temp2(0,1) = estimate_rel_dist_j.y/std::sqrt(d_j);
        temp2(1,1) = estimate_rel_dist_j.x/d_j;

        Hj = arma::join_rows(arma::join_rows(temp1,zeros_1j),arma::join_rows(temp2,zeros_1nj));

        // Noise
        R = arma::mat{2*n,2*n,arma::fill::eye}*R_noise;
        Rj = R.submat(j, j, j+1, j+1);
        
        // Kalman gain
        Ki = covariance_estimate*Hj.t()*(Hj*covariance_estimate*Hj.t() + Rj).i();

        // Update state to corrected prediction
        zai = zai_estimate + Ki*(zj - zj_hat);
        // zai(0) = normalize_angle(zai(0));
        // Update covariance
        covariance = (I - Ki*Hj)*covariance_estimate;
    }

    Robot_configuration EKFSlam::EKFSlam_config()
    {
        return {zai(1), zai(2), zai(0)};
    }

    Robot_configuration EKFSlam::EKFSlam_config_predicted()
    {
        return {zai_estimate(1), zai_estimate(2), zai_estimate(0)};
    }

}
