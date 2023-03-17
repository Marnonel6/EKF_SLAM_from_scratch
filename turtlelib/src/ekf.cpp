#include <iostream>
#include <armadillo>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"

namespace turtlelib
{
EKFSlam::EKFSlam()
: zai{m + 2 * n, arma::fill::zeros}, covariance{m + 2 * n, m + 2 * n, arma::fill::zeros}
{
  initialize_covariance();
}

EKFSlam::EKFSlam(Robot_configuration robot_config)
: zai{m + 2 * n, arma::fill::zeros}, covariance{m + 2 * n, m + 2 * n, arma::fill::zeros}
{
  initialize_covariance();
  initialize_robot_state(robot_config);
}

void EKFSlam::initialize_covariance()
{
  arma::mat cov_state, cov_obstacles, cov_zero1, cov_zero2;
  cov_state = arma::mat {m, m, arma::fill::zeros};
  cov_obstacles = arma::mat {2 * n, 2 * n, arma::fill::eye} *1000000.0;
  cov_zero1 = arma::mat {m, 2 * n, arma::fill::zeros};
  cov_zero2 = arma::mat {2 * n, m, arma::fill::zeros};
  covariance =
    arma::join_rows(
    arma::join_cols(cov_state, cov_zero2), arma::join_cols(
      cov_zero1,
      cov_obstacles));
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

  zai(0) = zai(0) + ut(0);
  zai(1) = zai(1) + ut(1);
  zai(2) = zai(2) + ut(2);


  // CALCULATE At matrix
  arma::mat zero_3_2n{m, 2 * n, arma::fill::zeros};
  arma::mat zero_2n_2n{2 * n, 2 * n, arma::fill::zeros};
  arma::mat temp(m, m, arma::fill::zeros);
  if (almost_equal(ut(0), 0.0)) {     // Zero rotational velocity
    temp(1, 0) = -(ut(1)) * sin(zai(0));
    temp(2, 0) = ut(1) * cos(zai(0));
    At = I +
      arma::join_rows(
      arma::join_cols(temp, zero_3_2n.t()),
      arma::join_cols(zero_3_2n, zero_2n_2n));
  } else {   // Non-zero rotational velocity
    temp(1, 0) = -(ut(1) / ut(0)) * cos(zai(0)) + (ut(1) / ut(0)) * cos(zai(0) + ut(0));
    temp(2, 0) = -(ut(1) / ut(0)) * sin(zai(0)) + (ut(1) / ut(0)) * sin(zai(0) + ut(0));
    At = I +
      arma::join_rows(
      arma::join_cols(temp, zero_3_2n.t()),
      arma::join_cols(zero_3_2n, zero_2n_2n));
  }

  Q_bar =
    arma::join_rows(arma::join_cols(Q, zero_3_2n.t()), arma::join_cols(zero_3_2n, zero_2n_2n));
  covariance = At * covariance * At.t() + Q_bar;
}

void EKFSlam::EKFSlam_Correct(double x, double y, size_t j)
{
  // Convert relative measurements to range bearing
  double r_j = std::sqrt(x * x + y * y);
  double phi_j = std::atan2(y, x);      // Normalize ?? TODO ??

  // Check if landmark has been seen before
  if (seen_landmarks.find(j) == seen_landmarks.end()) {
    // Initialize the landmark estimate x and y coordinates in zai
    zai(m + 2 * j) = zai(1) + r_j * cos(phi_j + zai(0));
    zai(m + 2 * j + 1) = zai(2) + r_j * sin(phi_j + zai(0));
    // Insert the new landmark index in the unordered_set
    seen_landmarks.insert(j);
  }

  // Actual measurements
  zj(0) = r_j;
  zj(1) = phi_j;

  // Estimate measurements
  Vector2D estimate_rel_dist_j;
  estimate_rel_dist_j.x = zai(m + 2 * j) - zai(1);
  estimate_rel_dist_j.y = zai(m + 2 * j + 1) - zai(2);
  double d_j = estimate_rel_dist_j.x * estimate_rel_dist_j.x + estimate_rel_dist_j.y *
    estimate_rel_dist_j.y;
  double r_j_hat = std::sqrt(d_j);
  double phi_j_hat = normalize_angle(atan2(estimate_rel_dist_j.y, estimate_rel_dist_j.x) - zai(0));
  zj_hat(0) = r_j_hat;
  zj_hat(1) = phi_j_hat;

  // Calculate H matrix
  arma::mat zeros_1j(2, 2 * j);
  arma::mat zeros_1nj(2, 2 * n - 2 * (j + 1));
  arma::mat temp1(2, 3);
  arma::mat temp2(2, 2);

  temp1(1, 0) = -1;
  temp1(0, 1) = -estimate_rel_dist_j.x / std::sqrt(d_j);
  temp1(1, 1) = estimate_rel_dist_j.y / d_j;
  temp1(0, 2) = -estimate_rel_dist_j.y / std::sqrt(d_j);
  temp1(1, 2) = -estimate_rel_dist_j.x / d_j;

  temp2(0, 0) = estimate_rel_dist_j.x / std::sqrt(d_j);
  temp2(1, 0) = -estimate_rel_dist_j.y / d_j;
  temp2(0, 1) = estimate_rel_dist_j.y / std::sqrt(d_j);
  temp2(1, 1) = estimate_rel_dist_j.x / d_j;

  Hj = arma::join_rows(arma::join_rows(temp1, zeros_1j), arma::join_rows(temp2, zeros_1nj));

  // Noise
  R = arma::mat{2 * n, 2 * n, arma::fill::eye} *R_noise;
  Rj = R.submat(j, j, j + 1, j + 1);

  // Kalman gain
  Ki = covariance * Hj.t() * (Hj * covariance * Hj.t() + Rj).i();

  // Update state to corrected prediction
  zai = zai + Ki * (zj - zj_hat);

  // Update covariance
  covariance = (I - Ki * Hj) * covariance;
}

size_t EKFSlam::Data_association(double x, double y)
{
  // Convert relative measurements to range bearing
  double r_j = std::sqrt(x * x + y * y);
  double phi_j = std::atan2(y, x);      // Normalize ?? TODO ??

  // Create a temp zai with new temp landmark
  zai_temp = zai;
  // Add N+1
  // Initialize the landmark estimate x and y coordinates in zai
  zai_temp(m + 2*N + 1) = zai_temp(1) + r_j * cos(phi_j + zai_temp(0));
  zai_temp(m + 2*N + 1 + 1) = zai_temp(2) + r_j * sin(phi_j + zai_temp(0));


  // Actual measurements
  zj(0) = r_j;
  zj(1) = phi_j;

  std::vector<arma::mat> distances{}; // Mahalanobis distance for each landmark

  for (int k = 0; k < N+1; k++)
  {
    /*
        Step 4.1.1
    */
    // Estimate measurements
    Vector2D estimate_rel_dist_j;
    estimate_rel_dist_j.x = zai_temp(m + 2 * k) - zai_temp(1);
    estimate_rel_dist_j.y = zai_temp(m + 2 * k + 1) - zai_temp(2);
    double d_j = estimate_rel_dist_j.x * estimate_rel_dist_j.x + estimate_rel_dist_j.y *
        estimate_rel_dist_j.y;

    // Calculate H matrix
    arma::mat zeros_1j(2, 2 * k);
    arma::mat zeros_1nj(2, 2 * n - 2 * (k + 1));
    arma::mat temp1(2, 3);
    arma::mat temp2(2, 2);

    temp1(1, 0) = -1;
    temp1(0, 1) = -estimate_rel_dist_j.x / std::sqrt(d_j);
    temp1(1, 1) = estimate_rel_dist_j.y / d_j;
    temp1(0, 2) = -estimate_rel_dist_j.y / std::sqrt(d_j);
    temp1(1, 2) = -estimate_rel_dist_j.x / d_j;

    temp2(0, 0) = estimate_rel_dist_j.x / std::sqrt(d_j);
    temp2(1, 0) = -estimate_rel_dist_j.y / d_j;
    temp2(0, 1) = estimate_rel_dist_j.y / std::sqrt(d_j);
    temp2(1, 1) = estimate_rel_dist_j.x / d_j;

    arma::mat Hk = arma::join_rows(arma::join_rows(temp1, zeros_1j), arma::join_rows(temp2, zeros_1nj));


    /*
        Step 4.1.2
    */
    // Noise
    R = arma::mat{2 * n, 2 * n, arma::fill::eye} *R_noise;
    arma::mat Rk = R.submat(k, k, k + 1, k + 1);

    arma::mat covariance_k = Hk*covariance*Hk.t() + Rk;


    /*
        Step 4.1.3
    */
    double r_j_hat = std::sqrt(d_j);
    double phi_j_hat = normalize_angle(atan2(estimate_rel_dist_j.y, estimate_rel_dist_j.x) - zai_temp(0));
    zj_hat(0) = r_j_hat;
    zj_hat(1) = phi_j_hat;


    /*
        Step 4.1.4 -> Calc Mahalanobis distance
    */
    arma::mat dist_k = ((zj - zj_hat).t())*(covariance_k.i())*(zj - zj_hat);

    distances.push_back(dist_k);
  }

  /*
      Step 4.2 & 4.3
  */
  // Set distance threshold to distance N+1
  arma::mat distance_threshold = distances.at(distances.size()-1);
  size_t index = N+1;
  bool new_landmark = true;

  for (size_t i = 0; i<distances.size(); i++)
  {
    if (distances.at(i)(0) < distance_threshold(0))
    {
        distance_threshold = distances.at(i);
        index = i;
        new_landmark = false; // Not a new landmark
    }
  }

  /*
      Step 4.4
  */
  if (new_landmark == true) // If it is a new landmark increase N
  {
    N++;
  }

  return index;
}

Robot_configuration EKFSlam::EKFSlam_config()
{
  return {zai(1), zai(2), zai(0)};
}

arma::colvec EKFSlam::EKFSlam_zai()
{
  return zai;
}
}
