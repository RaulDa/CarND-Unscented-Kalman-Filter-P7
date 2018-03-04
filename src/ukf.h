#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  // state dimension
  int n_x_;

  // augmented state dimension
  int n_aug_;

  // radar measurement dimension
  int n_z_;

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;

  // generated augmented sigma points matrix
  MatrixXd Xsig_aug_;

  // predicted sigma points matrix
  MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // radar measurement noise standard deviation radius in m
  double std_radr_;

  // radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // weights of sigma points
  VectorXd weights_;

  // sigma point spreading parameter
  double lambda_;

  // H matrix laser
  MatrixXd H_laser_;

  // time in seconds
  float time_s_;

  // vectors for storing data used with plotting purposes
  std::vector<float> queue_radar_nis_;
  std::vector<float> queue_laser_nis_;

  std::vector<float> yaw_estimate_;
  std::vector<float> yaw_ground_;

  std::vector<float> yaw_rate_estimate_;
  std::vector<float> yaw_rate_ground_;

  std::vector<float> vector_time_s_;
  std::vector<float> vector_radar_time_s_;
  std::vector<float> vector_lidar_time_s_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Calculates the augmented sigma points for the prediction step
   */
  void AugmentedSigmaPoints();

  /**
   * Predict sigma points during the prediction step
   * @param delta_t Time difference between actual and previous timestamps
   */
  void SigmaPointPrediction(double delta_t);

  /**
   * Predict mean and covariance using the predicted sigma points
   */
  void PredictMeanAndCovariance();

  /**
   * Measurement prediction during update step for radar
   * @param Zsig_out Predicted sigma points
   * @param z_out Predicted mean
   * @param S_out Predicted covariance matrix
   */
  void PredictRadarMeasurement(MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out);

  /**
   * Update step for radar
   * @param z Current radar measurement
   * @param Zsig_pred Sigma points
   * @param z_pred Mean
   * @param S_out Covariance matrix
   */
  void UpdateRadarState(VectorXd z, MatrixXd Zsig_pred, VectorXd z_pred, MatrixXd S);

  /**
   * Measurement prediction during update step for lidar
   * @param Zsig_out Predicted sigma points
   * @param z_out Predicted mean
   * @param S_out Predicted covariance matrix
   */
  void PredictLidarMeasurement(MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out);

  /**
   * Update step for lidar
   * @param z Current radar measurement
   * @param Zsig_pred Sigma points
   * @param z_pred Mean
   * @param S_out Covariance matrix
   */
  void UpdateLidarState(VectorXd z, MatrixXd Zsig_pred, VectorXd z_pred, MatrixXd S);

private:

  // tool object used to adjust angle
  Tools tools;

};

#endif /* UKF_H */
