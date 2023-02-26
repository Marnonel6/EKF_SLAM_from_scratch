#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP
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

    /// \brief Kinematics of a differential drive robot.
    class EKF
    {
    private:
        /// \brief State of the robot at time t
        // arma::colvec qt{m,arma::fill::zeros};
        // arma::colvec mt{n,arma::fill::zeros};
        /// \brief State of the robot at time t
        arma::colvec zai{};
        /// \brief Covariance
        arma::mat covariance{};
        // /// \brief
        // arma::mat At{m+2*n,m+2*n,arma::fill::zeros};

    public:
        /// \brief start at origin and default the uncertainty
        EKF();

        /// \brief set robot start config and default the uncertainty
        /// \param robot_config - robot start configuration
        explicit EKF(Robot_configuration robot_config);

        /// \brief set the initial guess covariance matrix
        void initialize_covariance();

        /// \brief set the initial state of the robot
        /// \param robot_config - robot start configuration
        void initialize_robot_state(Robot_configuration robot_config);
    };
}

#endif
