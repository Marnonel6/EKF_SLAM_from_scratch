#ifndef EKFSlam_INCLUDE_GUARD_HPP
#define EKFSlam_INCLUDE_GUARD_HPP
/// \file
/// \brief Extended Kalman Filter (SLAM).

#include <iosfwd>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>

namespace turtlelib
{
    /// \brief maximum number of obstacles
    constexpr int n=20;
    /// \brief size of robot state vector
    constexpr int m=3;
    /// \brief process noise
    constexpr double wt = 0.0;

    /// \brief Kinematics of a differential drive robot.
    class EKFSlam
    {
    private:
        /// \brief State of the robot at time t
        // arma::colvec qt{m,arma::fill::zeros};
        // arma::colvec mt{n,arma::fill::zeros};
        /// \brief State of the robot at time t
        arma::colvec zai{};
        /// \brief Covariance
        arma::mat covariance{};
        /// \brief Estimate state of the robot at time t
        arma::colvec zai_estimate{m+2*n,arma::fill::zeros};
        /// \brief Covariance estimate
        arma::mat covariance_estimate{m+2*n,m+2*n,arma::fill::zeros};
        /// \brief Given twist
        arma::colvec ut{m};
        /// \brief Previous twist
        Twist2D prev_twist;
        /// \brief Identity matrix
        arma::mat I{m+2*n,m+2*n,arma::fill::eye};
        /// \brief At matrix
        arma::mat At{m+2*n,m+2*n,arma::fill::zeros};
        /// \brief Process noise for the robot motion model
        arma::mat Q{arma::mat{m,m,arma::fill::eye}*wt};
        /// \brief Process noise for robot motion model
        arma::mat Q_bar{m+2*n,m+2*n,arma::fill::zeros};

    public:
        /// \brief start at origin and default the uncertainty
        EKFSlam();

        /// \brief set robot start config and default the uncertainty
        /// \param robot_config - robot start configuration
        explicit EKFSlam(Robot_configuration robot_config);

        /// \brief set the initial guess covariance matrix
        void initialize_covariance();

        /// \brief set the initial state of the robot
        /// \param robot_config - robot start configuration
        void initialize_robot_state(Robot_configuration robot_config);

        /// \brief predict/estimate the robot state and propogate the uncertainty
        /// \param twist - twist control at time t
        void EKFSlam_Predict(Twist2D twist);
    };
}

#endif
