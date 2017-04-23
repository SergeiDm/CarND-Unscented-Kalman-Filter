#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Initialize Unscented Kalman filter
UKF::UKF() {
  // Initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // If this is false, laser measurements will be ignored
  use_laser_ = true;

  // If this is false, radar measurements will be ignored
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Initial state vector
  x_ = VectorXd(n_x_);

  // Initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_ <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
        -0.0013,    0.0077,    0.0011,    0.0080,    0.0060,
         0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
        -0.0022,    0.0080,    0.0007,    0.0098,    0.0100,
        -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // Predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  previous_timestamp_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.54;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {
    weights_(i) = 0.5/(n_aug_+lambda_);
  }
}

UKF::~UKF() {}

// Angle normalization
void UKF::NormalizeAngle(double &angle) {
  if (angle > M_PI || angle < -M_PI) {
    angle = fmod(angle, 2*M_PI);
  }
}

// @param {MeasurementPackage} meas_package The latest measurement data of
// either radar or laser.
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  // Initialization
  if (!is_initialized_) {
    // The state initialization
    x_ << 0, 0, 0, 0, 0;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      float ro = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      // Convert radar from polar to cartesian coordinates
      x_(0) = ro*cos(phi);
      x_(1) = ro*sin(phi);
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
    }

    previous_timestamp_ = meas_package.timestamp_;

    is_initialized_ = true;
    return;
  }
  // Prediction
  float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  if (dt == 0) {
      dt = 1e-5;
  }

	previous_timestamp_ = meas_package.timestamp_;

  // Prediction step
  Prediction(dt);

  // Update step
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  }
}

// Predicts sigma points, the state, and the state covariance matrix.
// @param {double} delta_t the change in time (in seconds) between the last
// measurement and this one.
void UKF::Prediction(double delta_t) {

  // Create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // Create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // Create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // Create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  // Predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++) {
    // Extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // Predicted state values
    double px_p, py_p;

    // Avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // Add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;

    yawd_p = yawd_p + nu_yawdd*delta_t;

    // Write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  // Create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  // Create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  // Predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x+ weights_(i) * Xsig_pred_.col(i);
  }

  // Predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;

    // Angle normalization
    NormalizeAngle(x_diff(3));

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  // Write results
  x_ = x;
  P_ = P;
}

// Updates the state and the state covariance matrix using a laser measurement.
// @param {MeasurementPackage} meas_package
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  // Set measurement dimension, lidar can measure px, py
  int n_z = 2;

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // Transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Zsig(0,i) = Xsig_pred_(0,i);
    Zsig(1,i) = Xsig_pred_(1,i);
  }

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;
  S = S + R;

  // Create example vector for incoming laser measurement
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0],
       meas_package.raw_measurements_[1];

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // Calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  // Residual
  VectorXd z_diff = z - z_pred;

  // Update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // Calculate NIS
  NIS_laser_ =z_diff.transpose()*Si*z_diff;
}

// Updates the state and the state covariance matrix using a radar measurement.
// @param {MeasurementPackage} meas_package
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  // Set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // Transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // Measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi

    if (p_x == 0 && p_y == 0) {
      p_x = 1e-8;
      p_y = 1e-8;
    }
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Angle normalization
    NormalizeAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;

  // Create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0],
       meas_package.raw_measurements_[1],
       meas_package.raw_measurements_[2];

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // Calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Angle normalization
    NormalizeAngle(z_diff(1));

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalization
    NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  // Residual
  VectorXd z_diff = z - z_pred;

  // Angle normalization
  NormalizeAngle(z_diff(1));

  // Update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // Calculate NIS
  NIS_radar_ =z_diff.transpose()*Si*z_diff;
}
