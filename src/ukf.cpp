#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

#define NUM_STATES 5

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */


	if (!is_initialized_) {
		Initalize(meas_package);
		is_initialized_ = true;
		return;
	}

	if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_) {
		return;
	}
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) {
		return;
	}
	//find time difference then update previous time stamp
	double current_time_s = (double)meas_package.timestamp_ / 1000000;
	double dt_s = current_time_s - previous_timestamp_;
	previous_timestamp_ = current_time_s;

	//Predict
	Prediction(dt_s);

	//Update
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		UpdateRadar(meas_package);
	}
	else {
		UpdateLidar(meas_package);
	}

}

void UKF::Initalize(MeasurementPackage meas_package) {
	
	// first measurement
	cout << "UKF: " << endl;
	x_ = VectorXd(NUM_STATES);
	x_ << 1, 1, 1, 1,1;

	//declare state variables
	double px0;
	double py0;
	double v;
	double yaw;
	double yawRate;

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		/**
		Convert radar from polar to cartesian coordinates and initialize state.
		*/

		double rho = meas_package.raw_measurements_[0];
		double theta = meas_package.raw_measurements_[1];

		px0 = rho * cos(theta);
		py0 = rho * sin(theta);
		v = 0;
		yaw = 0;
		yawRate = 0;

	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		/**
		Initialize state.
		*/
		double px = meas_package.raw_measurements_[0];
		double py = meas_package.raw_measurements_[1];

		px0 = px;
		py0 = py;
		v = 0;
		yaw = 0;
		yawRate = 0;

	}
	x_ << px0, py0, v, yaw, yawRate;  //write state variables
	previous_timestamp_ = (double)meas_package.timestamp_ / 1000000;  //store time_stamp

	//Initialize state covariance matrix
	P_ = MatrixXd(NUM_STATES, NUM_STATES);
	P_ << 1, 0, 0   , 0  , 0,
	      0, 1, 0   , 0  , 0,
		  0, 0, 1000, 0  , 0,
		  0, 0, 0   ,1000, 0,
		  0, 0, 0   , 0  , 1000;

	return;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

	GenerateSigmaPoints();
	SigmaPointPrediction(delta_t);
	CalculateWeights();
	PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

	PredictRadarMeasurement();
	UpdateState(meas_package);
}

void UKF::GenerateSigmaPoints() {

	//set state dimension
	int n_x = NUM_STATES;

	//set augmented dimension
	int n_aug = n_x + 2;

	//define spreading parameter
	double lambda = 3 - n_aug;


	//create augmented mean vector
	VectorXd x_aug = VectorXd(7);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(7, 7);

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

	/*******************************************************************************
	* Student part begin
	******************************************************************************/

	//create augmented mean state
	x_aug.head(n_x) = x_;
	x_aug(n_x) = 0;
	x_aug(n_x + 1) = 0;

	//create augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(n_x, n_x) = P_;
	P_aug(n_x, n_x) = std_a_ * std_a_;
	P_aug(n_x + 1, n_x + 1) = std_yawdd_ * std_yawdd_;

	//create square root matrix
	MatrixXd A = P_aug.llt().matrixL();
	//create augmented sigma points
	MatrixXd lnxa = sqrt(lambda + n_aug) *A;

	Xsig_aug.col(0) = x_aug;

	for (int i = 0; i < n_aug; i++)
	{
		//std::cout << x+ lnxa.col(i) << std::endl;
		Xsig_aug.col(i + 1) = x_aug + lnxa.col(i);
		Xsig_aug.col(n_aug + i + 1) = x_aug - lnxa.col(i);

	}

	Xsig_aug_ = Xsig_aug;


}

void UKF::SigmaPointPrediction(double delta_t) {

	//set state dimension
	int n_x = NUM_STATES;

	//set augmented dimension
	int n_aug = n_x + 2;


	//create matrix with predicted sigma points as columns
	MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);



	//predict sigma points
	for (int i = 0; i < 2 * n_aug + 1; i++)
	{
		double px = Xsig_aug_(0, i);
		double py = Xsig_aug_(1, i);
		double v = Xsig_aug_(2, i);
		double yaw = Xsig_aug_(3, i);
		double yaw_d = Xsig_aug_(4, i);
		double noise_a = Xsig_aug_(5, i);
		double noise_yaw_dd = Xsig_aug_(6, i);

		double px_new = 0;
		double py_new = 0;
		double v_new = 0;
		double yaw_new = 0;
		double yaw_d_new = 0;


		//avoid division by zero
		if (fabs(yaw_d) < 0.001)
		{
			px_new = px + v * cos(yaw)*delta_t;
			py_new = py + v * sin(yaw)*delta_t;
		}
		else
		{
			px_new = px + v / yaw_d * (sin(yaw + yaw_d * delta_t) - sin(yaw));
			py_new = py + v / yaw_d * (-cos(yaw + yaw_d * delta_t) + cos(yaw));
		}

		v_new = v;
		yaw_new = yaw + yaw_d * delta_t;
		yaw_d_new = yaw_d;

		//Add noise
		//std::cout <<"index:" << i << std::endl;
		//std::cout <<"px_new:" << px_new << std::endl;
		px_new = px_new + 0.5*delta_t*delta_t*cos(yaw)*noise_a;
		py_new = py_new + 0.5*delta_t*delta_t*sin(yaw)*noise_a;
		v_new = v_new + delta_t * noise_a;
		yaw_new = yaw_new + 0.5*delta_t*delta_t*noise_yaw_dd;
		yaw_d_new = yaw_d_new + delta_t * noise_yaw_dd;
		//std::cout <<"px_new +  noise:" <<  0.5*delta_t*delta_t*cos(yaw) << std::endl;
		//write predicted sigma points into right column

		Xsig_pred(0, i) = px_new;
		Xsig_pred(1, i) = py_new;
		Xsig_pred(2, i) = v_new;
		Xsig_pred(3, i) = yaw_new;
		Xsig_pred(4, i) = yaw_d_new;
	}

	Xsig_pred_ = Xsig_pred;

}

void UKF::CalculateWeights(void)
{

	//set state dimension
	int n_x = NUM_STATES;

	//set augmented dimension
	int n_aug = n_x + 2;

	//define spreading parameter
	double lambda = 3 - n_aug;


	weights_ = VectorXd(2 * n_aug + 1);
	//set weights
	weights_(0) = lambda / (lambda + n_aug);
	for (int i = 0; i < 2 * n_aug; i++)
	{
		weights_(i + 1) = 0.5 / (lambda + n_aug);
	}
}

void UKF::PredictMeanAndCovariance() {

	//set state dimension
	int n_x = NUM_STATES;

	//set augmented dimension
	int n_aug = n_x + 2;

	//define spreading parameter
	double lambda = 3 - n_aug;

	//create vector for predicted state
	VectorXd x = VectorXd(n_x);

	//create covariance matrix for prediction
	MatrixXd P = MatrixXd(n_x, n_x);

	
	//predict state mean
	x.fill(0.0);
	for (int i = 0; i < 2 * n_aug + 1; i++)
	{
		//std::cout << x << std::endl;
		//std::cout << weights_ << std::endl;
		//std::cout << Xsig_pred_ << std::endl;
		x = x + weights_(i) * Xsig_pred_.col(i);
		//std::cout << x << std::endl;
	}
	//predict state covariance matrix
	P.fill(0.0);
	for (int i = 0; i < 2 * n_aug + 1; i++)
	{
		VectorXd error = Xsig_pred_.col(i) - x;
		//angle normalization
		while (error(3) > M_PI) error(3) -= 2.*M_PI;
		while (error(3) < -M_PI) error(3) += 2.*M_PI;

		P = P + weights_(i) * error*error.transpose();
	}

	x_ = x;
	P_ = P;
}

void UKF::PredictRadarMeasurement() {

	//set measurement dimension, radar can measure r, phi, and r_dot
	int n_z = 3;

	//set state dimension
	int n_x = NUM_STATES;

	//set augmented dimension
	int n_aug = n_x + 2;

	//define spreading parameter
	double lambda = 3 - n_aug;



	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);


	//transform sigma points into measurement space
	for (int i = 0; i < 2 * n_aug + 1; i++)
	{
		double px = Xsig_pred_(0, i);
		double py = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);
		double yaw_d = Xsig_pred_(4, i);

		double radar_dist;
		double radar_angle;
		double radar_velocity;

		radar_dist = sqrt(px*px + py * py);
		radar_angle = atan2(py, px);
		radar_velocity = px * cos(yaw)*v + py * sin(yaw)*v;
		if (radar_dist < 0.001)
		{
			radar_velocity = radar_velocity / 0.001;
		}
		else
		{
			radar_velocity = radar_velocity / radar_dist;
		}

		Zsig(0, i) = radar_dist;
		Zsig(1, i) = radar_angle;
		Zsig(2, i) = radar_velocity;
	}

	//calculate mean predicted measurement
	z_pred.fill(0.0);
	for (int i = 0; i < 2 * n_aug + 1; i++)
	{
		z_pred = z_pred + weights_(i) * Zsig.col(i);
		//std::cout << x << std::endl;
	}

	//calculate innovation covariance matrix S
	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug + 1; i++)
	{
		VectorXd error = Zsig.col(i) - z_pred;
		//angle normalization
		while (error(1) > M_PI) error(1) -= 2.*M_PI;
		while (error(1) < -M_PI) error(1) += 2.*M_PI;

		S = S + weights_(i) * error*error.transpose();
	}

	MatrixXd R = MatrixXd(n_z, n_z);
	R.fill(0.0);
	R(0, 0) = std_radr_ * std_radr_;
	R(1, 1) = std_radphi_ * std_radphi_;
	R(2, 2) = std_radrd_ * std_radrd_;

	S = S + R;

	//update results
	z_pred_ = z_pred;
	S_ = S;
	Zsig_ = Zsig;

}

void UKF::UpdateState(MeasurementPackage meas_package) {

	int n_z;
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		n_z = 3;
	}
	else {
		n_z = 2;
	}
	//set measurement dimension, radar can measure r, phi, and r_dot
	

	//set state dimension
	int n_x = NUM_STATES;

	//set augmented dimension
	int n_aug = n_x + 2;

	//define spreading parameter
	double lambda = 3 - n_aug;

	VectorXd z = meas_package.raw_measurements_;

	
	MatrixXd Tc = MatrixXd(n_x, n_z);


	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug + 1; i++)
	{
		VectorXd predictionDiff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (predictionDiff(3) > M_PI) predictionDiff(3) -= 2.*M_PI;
		while (predictionDiff(3) < -M_PI) predictionDiff(3) += 2.*M_PI;

		VectorXd measDiff = Zsig_.col(i) - z_pred_;
		//angle normalization
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			while (measDiff(1) > M_PI) measDiff(1) -= 2.*M_PI;
			while (measDiff(1) < -M_PI) measDiff(1) += 2.*M_PI;
		}

		Tc = Tc + weights_(i)*predictionDiff*measDiff.transpose();
	}
	//calculate Kalman gain K;
	MatrixXd K = Tc * S_.inverse();

	//update state mean and covariance matrix
	//residual
	VectorXd z_diff = z - z_pred_;

	//angle normalization
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;
	}

	x_ = x_ + K * (z_diff);
	P_ = P_ - K * S_*K.transpose();
}
