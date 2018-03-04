#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = 7;

  // radar measurement dimension
  n_z_ = 3;

  // will be set to true during processing of 1st measurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_ << 0.0225, 0, 0, 0, 0,
		0, 0.0225, 0, 0, 0,
		0, 0, 1, 0, 0,
		0, 0, 0, 1, 0,
		0, 0, 0, 0, 1;

  // generated augmented sigma points
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // time when the state is true, in us
  time_us_ = 0;

  // process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/3;
  
  // laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  // weights vector
  weights_ = VectorXd(2 * n_aug_ + 1);

  // lambda parameter
  lambda_ = 3 - n_x_;

  // H matrix laser
  H_laser_ = MatrixXd(2, 5);
  H_laser_ << 1, 0, 0, 0, 0,
  		     0, 1, 0, 0, 0;

  // time in seconds
  time_s_ = 0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  // initialization
  if (!is_initialized_) {

	 // first measurement
	 cout << "EKF: " << endl;

	 // initialize state vector depending on whether first measurement is radar or laser
	 if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

	    // convert radar from polar to cartesian coordinates and initialize state.
	    x_ << (meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1])), (meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1])), 0, 0, 0;
	 }
	 else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

	    // initialize state.
	    x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
	 }

	 // store current timestamp for use in next cycle
	 time_us_ = meas_package.timestamp_;

	 // done initializing, no need to predict or update
	 is_initialized_ = true;
	 return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // calculate time difference between measurements and store result
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // obtain actual time and store in vector for plotting purposes
  time_s_ = time_s_ + dt;
  vector_time_s_.push_back(time_s_);

  // generate augmented sigma points
  AugmentedSigmaPoints();

  // predict
  Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // update in function of radar or laser
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
	  // do radar update
	  UpdateRadar(meas_package);
  } else {
	  // do lidar update
	  UpdateLidar(meas_package);
  }

  // print the output
  cout << "x_ = " << x_.transpose() << endl;
  cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

	// call function to predict sigma points
	SigmaPointPrediction(delta_t);

	// call function to predict mean and covariance from predicted sigma points
	PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  MatrixXd Zsig = MatrixXd(2, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(2);
  MatrixXd S = MatrixXd(2, 2);

  // call predict measurement and update functions for lidar
  PredictLidarMeasurement(&Zsig, &z_pred, &S);
  UpdateLidarState(meas_package.raw_measurements_, Zsig, z_pred, S);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(3);
  MatrixXd S = MatrixXd(3, 3);

  // call predict measurement and update functions for radar
  PredictRadarMeasurement(&Zsig, &z_pred, &S);
  UpdateRadarState(meas_package.raw_measurements_, Zsig, z_pred, S);
}

/**
 * Calculates the augmented sigma points for the prediction step
 */
void UKF::AugmentedSigmaPoints() {

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug_.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++){
	  Xsig_aug_.col(i+1) = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
	  Xsig_aug_.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
  }

}

/**
 * Predict sigma points during the prediction step
 * @param delta_t Time difference between actual and previous timestamps
 */
void UKF::SigmaPointPrediction(double delta_t) {

  // predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    // extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin(yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);

    v_p = v_p + nu_a*delta_t;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

}

/**
 * Predict mean and covariance using the predicted sigma points
 */
void UKF::PredictMeanAndCovariance() {

  // set weights
  weights_(0) = lambda_/(lambda_+n_aug_);

  for(int i=1;i<2*n_aug_+1;i++){
      weights_(i) = 1/(2*(lambda_+n_aug_));
  }

  // predict state mean
  x_.fill(0.0);
  for (int i=0;i<2*n_aug_+1;i++){
      x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  // predict state covariance matrix
  P_.fill(0.0);
  for (int i=0;i<2*n_aug_+1;i++){

	  // state difference
	  VectorXd x_diff = Xsig_pred_.col(i) - x_;
	  // angle normalization
	  tools.AdjustAngle(3, x_diff);

      P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

}

/**
 * Measurement prediction during update step for radar
 * @param Zsig_out Predicted sigma points
 * @param z_out Predicted mean
 * @param S_out Predicted covariance matrix
 */
void UKF::PredictRadarMeasurement(MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out) {

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_,n_z_);

  // transform sigma points into measurement space
  for (int i=0;i<2*n_aug_+1;i++){
      Zsig(0, i) = sqrt(Xsig_pred_(0,i)*Xsig_pred_(0,i)+Xsig_pred_(1,i)*Xsig_pred_(1,i));
      Zsig(1, i) = atan2(Xsig_pred_(1,i),Xsig_pred_(0,i));
      Zsig(2, i) = (Xsig_pred_(0,i)*cos(Xsig_pred_(3,i))*Xsig_pred_(2,i)+Xsig_pred_(1,i)*sin(Xsig_pred_(3,i))*Xsig_pred_(2,i))/sqrt(Xsig_pred_(0,i)*Xsig_pred_(0,i)+Xsig_pred_(1,i)*Xsig_pred_(1,i));
  }

  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0;i<2*n_aug_+1;i++){
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  VectorXd dif = VectorXd(n_z_);
  S.fill(0.0);
  // calculate innovation covariance matrix S
  for (int i=0;i<2*n_aug_+1;i++){

      dif = Zsig.col(i) - z_pred;

      tools.AdjustAngle(1, dif);

      S = S + weights_(i) * dif * dif.transpose();
  }

  // calculate noise matrix R
  MatrixXd R = MatrixXd(n_z_,n_z_);
  R.fill(0.0);
  R(0,0) = std_radr_*std_radr_;
  R(1,1) = std_radphi_*std_radphi_;
  R(2,2) = std_radrd_*std_radrd_;

  // add noise matrix to covariance matrix S
  S = S + R;

  //write result
  *Zsig_out = Zsig;
  *z_out = z_pred;
  *S_out = S;
}

/**
 * Update step for radar
 * @param z Current radar measurement
 * @param Zsig_pred Sigma points
 * @param z_pred Mean
 * @param S_out Covariance matrix
 */
void UKF::UpdateRadarState(VectorXd z, MatrixXd Zsig_pred, VectorXd z_pred, MatrixXd S) {

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // residual
    VectorXd z_diff = Zsig_pred.col(i) - z_pred;
    // angle normalization
    tools.AdjustAngle(1, z_diff);

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    tools.AdjustAngle(3, x_diff);

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  tools.AdjustAngle(1, z_diff);

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // calculate NIS and add it to radar NIS vector
  float radar_nis = z_diff.transpose() * S.inverse() * z_diff;
  queue_radar_nis_.push_back(radar_nis);
  vector_radar_time_s_.push_back(time_s_);

  // export NIS to text file
  if (queue_radar_nis_.size()>245){
	  ofstream myfile;
	  myfile.open("radar.txt");

    for (int i=0;i<queue_radar_nis_.size();i++){
       myfile << vector_radar_time_s_[i] << " " << queue_radar_nis_[i] << " " << 7.815;
       myfile << "\n";
    }
    myfile.close();
  }

}

/**
 * Measurement prediction during update step for lidar
 * @param Zsig_out Predicted sigma points
 * @param z_out Predicted mean
 * @param S_out Predicted covariance matrix
 */
void UKF::PredictLidarMeasurement(MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out) {

  // dimension laser measurement
  int n_las_meas = 2;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_las_meas, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_las_meas);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_las_meas, n_las_meas);

  for (int i=0;i<2*n_aug_+1;i++){
      Zsig.col(i) = H_laser_ * Xsig_pred_.col(i);
  }

  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0;i<2*n_aug_+1;i++){
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  VectorXd dif = VectorXd(n_las_meas);
  S.fill(0.0);
  // calculate innovation covariance matrix S
  for (int i=0;i<2*n_aug_+1;i++){

      dif = Zsig.col(i) - z_pred;

      S = S + weights_(i) * dif * dif.transpose();
  }

  // calculate noise matrix R
  MatrixXd R = MatrixXd(n_las_meas,n_las_meas);
  R.fill(0.0);
  R(0,0) = std_laspx_*std_laspx_;
  R(1,1) = std_laspy_*std_laspy_;

  // add R to covariance matrix S
  S = S + R;

  //write result
  *Zsig_out = Zsig;
  *z_out = z_pred;
  *S_out = S;
}

/**
 * Update step for lidar
 * @param z Current radar measurement
 * @param Zsig_pred Sigma points
 * @param z_pred Mean
 * @param S_out Covariance matrix
 */
void UKF::UpdateLidarState(VectorXd z, MatrixXd Zsig_pred, VectorXd z_pred, MatrixXd S) {

  // dimension laser measurement
  int n_las_meas = 2;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_las_meas);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // residual
    VectorXd z_diff = Zsig_pred.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    tools.AdjustAngle(3, x_diff);

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // calculate NIS and add store value to vector
  float lidar_nis = z_diff.transpose() * S.inverse() * z_diff;
  queue_laser_nis_.push_back(lidar_nis);
  vector_lidar_time_s_.push_back(time_s_);

  // export NIS to text file
  if (queue_laser_nis_.size()>245){
	  ofstream myfile;
	  myfile.open("lidar.txt");

    for (int i=0;i<queue_laser_nis_.size();i++){
       myfile << vector_lidar_time_s_[i] << " " << queue_laser_nis_[i] << " " << 5.991;
       myfile << "\n";
    }
    myfile.close();
  }

}

