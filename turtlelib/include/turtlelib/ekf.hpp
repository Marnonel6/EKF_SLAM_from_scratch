#ifndef EKFSlam_INCLUDE_GUARD_HPP
#define EKFSlam_INCLUDE_GUARD_HPP
/// \file
/// \brief Extended Kalman Filter (SLAM).

#include <iosfwd>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include <unordered_set>

namespace turtlelib
{
    /// \brief maximum number of obstacles
    constexpr int n=10;
    /// \brief size of robot state vector
    constexpr int m=3;
    /// \brief process noise
    constexpr double wt = 0.001;
    /// \brief noise on landmarks
    constexpr double R_noise = 0.01;

    /// \brief Landmark location
    struct Landmark
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
    };


    /// \brief Kinematics of a differential drive robot.
    class EKFSlam
    {
    private:
        /// \brief State of the robot at time t
        arma::colvec zai{};
        /// \brief Covariance
        arma::mat covariance{};
        /// \brief Given twist
        arma::colvec ut{m,arma::fill::zeros};
        /// \brief Previous twist
        Twist2D prev_twist{0.0,0.0,0.0};
        /// \brief Identity matrix
        arma::mat I{m+2*n,m+2*n,arma::fill::eye};
        /// \brief At matrix
        arma::mat At{m+2*n,m+2*n,arma::fill::zeros};
        /// \brief Process noise for the robot motion model
        arma::mat Q{arma::mat{m,m,arma::fill::eye}*wt};
        /// \brief Process noise for robot motion model
        arma::mat Q_bar{m+2*n,m+2*n,arma::fill::zeros};
        /// \brief Previously seen landmark id's
        std::unordered_set<int> seen_landmarks{};
        /// \brief Actual measurement
        arma::colvec zj{2,arma::fill::zeros};
        /// \brief Estimate measurement
        arma::colvec zj_hat{2,arma::fill::zeros};
        /// \brief H matrix
        arma::mat Hj{};
        /// \brief Kalman gain
        arma::mat Ki{};
        /// \brief Noise
        arma::mat R{2*n,2*n,arma::fill::eye};
        /// \brief Noise for j landmark
        arma::mat Rj{};
        /// \brief Data association index
        int N = 0;
        /// \brief Temporary State of the robot at time t
        arma::colvec zai_temp{};

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

        /// \brief correction calculations
        /// \param x - landmark x-coordinate
        /// \param y - landmark y-coordinate
        /// \param j - landmark index j
        void EKFSlam_Correct(double x, double y, size_t j);

        /// \brief data association with the Mahalanobis distance
        /// \param x - landmark x-coordinate
        /// \param y - landmark y-coordinate
        /// \return landmark index j (size_t)
        size_t Data_association(double x, double y);

        /// \brief get SLAM corrected configuration
        Robot_configuration EKFSlam_config();

        /// \brief get zai from ekf slam update
        arma::colvec EKFSlam_zai();
    };
}

#endif
