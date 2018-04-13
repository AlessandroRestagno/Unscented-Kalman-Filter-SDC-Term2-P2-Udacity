#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
* Initializes Unscented Kalman filter
*/
UKF::UKF() {
	
	// Not initialized until first process measurement
	is_initialized_ = false;

	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// initial state vector
	x_ = VectorXd(5);

	// initial covariance matrix
	P_ = MatrixXd(5, 5);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 1.5;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = M_PI / 6 ;

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

	// Set state dimension
	n_x_ = 5;

	// Dimension of radar measurement space
	n_radar_ = 3;

	// Dimension of laser measurement space
	n_laser_ = 2;

	// Set augmented dimension
	n_aug_ = 7;

	// Number of sigma points
	n_sig_ = 2 * n_aug_ + 1;

	// Sigma point spreading parameter
	lambda_ = 3 - n_x_;

	// Augmented sigma points matrix
	Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

	// Matrix to hold sigma points
	Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

	// Vector for weights
	weights_ = VectorXd(2 * n_aug_ + 1);

	// Measurement noise covariance matrices initialization
	R_radar_ = MatrixXd(n_radar_, n_radar_);
	R_radar_.fill(0.);
	R_radar_(0, 0) = std_radr_ * std_radr_;
	R_radar_(1, 1) = std_radphi_ * std_radphi_;
	R_radar_(2, 2) = std_radrd_ * std_radrd_;

	R_lidar_ = MatrixXd(n_laser_, n_laser_);
	R_lidar_.fill(0.);
	R_lidar_(0, 0) = std_laspx_ * std_laspx_;
	R_lidar_(1, 1) = std_laspy_ * std_laspy_;


	// Start time
	time_us_ = 0;

}
UKF::~UKF() {}

/**
* @param {MeasurementPackage} meas_package The latest measurement data of
* either radar or laser.
*/
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

	if (!is_initialized_) {

		
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

			double rho = meas_package.raw_measurements_(0);
			double phi = meas_package.raw_measurements_(1);
			double rhodot = meas_package.raw_measurements_(2);

			// polar to cartesian - r * cos(angle) for x and r * sin(angle) for y
			x_ << rho * cos(phi), rho * sin(phi), 3.25, rhodot * cos(phi), rhodot * sin(phi);

			//state covariance matrix
			P_ << std_radr_ * std_radr_, 0, 0, 0, 0,
				0, std_radr_*std_radr_, 0, 0, 0,
				0, 0, 1, 0, 0,
				0, 0, 0, std_radphi_, 0,
				0, 0, 0, 0, std_radphi_;
		}

		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			// Initialize state.
			// ***** Last three values below can be tuned *****
			x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 3, 0.75, 0;

			//state covariance matrix
			P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
				0, std_laspy_*std_laspy_, 0, 0, 0,
				0, 0, 1, 0, 0,
				0, 0, 0, 1, 0,
				0, 0, 0, 0, 1;
		}

		// Initialize weights
		weights_(0) = lambda_ / (lambda_ + n_aug_);
		for (int i = 1; i < weights_.size(); i++) {
			weights_(i) = 0.5 / (n_aug_ + lambda_);
		}

		// done initializing, no need to predict or update
		is_initialized_ = true;
		time_us_ = meas_package.timestamp_;
		return;

	}

	// Calculate delta_t, store current time for future
	double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;

	// Predict
	Prediction(delta_t);

	// Measurement updates
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		UpdateRadar(meas_package);
	}
	else {
		UpdateLidar(meas_package);
	}

}

/**
* Predicts sigma points, the state, and the state covariance matrix.
* @param {double} delta_t the change in time (in seconds) between the last
* measurement and this one.
*/


void UKF::Prediction(double delta_t) {

	/*******************************************************************************
	* Augmentation
	******************************************************************************/
	
	int n_sig = 2 * n_aug_ + 1;

	//create augmented mean vector
	VectorXd x_aug = VectorXd(7);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(7, 7);

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig);

	//create augmented mean state
	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	//create augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(5, 5) = P_;
	P_aug(5, 5) = std_a_ * std_a_;
	P_aug(6, 6) = std_yawdd_ * std_yawdd_;

	//create square root matrix
	MatrixXd A = P_aug.llt().matrixL();
	MatrixXd B = sqrt(lambda_ + n_aug_) * A;

	//create augmented sigma points
	Xsig_aug.col(0) = x_aug;
	Xsig_aug.leftCols(n_aug_ + 1).rightCols(n_aug_) = x_aug.replicate(1, B.cols()) + B;
	Xsig_aug.rightCols(n_aug_) = x_aug.replicate(1, B.cols()) - B;

	/*******************************************************************************
	* Prediction
	******************************************************************************/

	//predict sigma points
	for (int i = 0; i< n_sig; i++) {
		//extract values for better readability
		double p_x = Xsig_aug(0, i);
		double p_y = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double yaw = Xsig_aug(3, i);
		double yawd = Xsig_aug(4, i);
		double nu_a = Xsig_aug(5, i);
		double nu_yawdd = Xsig_aug(6, i);

		double delta_t2 = delta_t * delta_t;
		double yawd_dt = yawd * delta_t;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			Xsig_pred_(0, i) = p_x + v / yawd * (sin(yaw + yawd_dt) - sin(yaw)) +
				(0.5 * delta_t2 * cos(yawd) * nu_a);
			Xsig_pred_(1, i) = p_y + v / yawd * (-cos(yaw + yawd_dt) + cos(yaw)) +
				(0.5 * delta_t2 * sin(yawd) * nu_a);
		}
		else {
			Xsig_pred_(0, i) = p_x + v * delta_t*cos(yaw) +
				(0.5 * delta_t2 * cos(yawd) * nu_a);
			Xsig_pred_(1, i) = p_y + v * delta_t*sin(yaw) +
				(0.5 * delta_t2 * sin(yawd) * nu_a);
		}

		Xsig_pred_(2, i) = v + nu_a * delta_t;
		Xsig_pred_(3, i) = yaw + yawd_dt + 0.5 * nu_yawdd * delta_t2;
		Xsig_pred_(4, i) = yawd + nu_yawdd * delta_t;
	}

	/*******************************************************************************
	* Predicted Mean and Covariance
	******************************************************************************/

	// Predict state mean
	MatrixXd x_sum = Xsig_pred_.array().rowwise() * weights_.transpose().array();
	x_ = x_sum.rowwise().sum();

	//predict state covariance matrix
	MatrixXd X_abs = Xsig_pred_.array().colwise() - x_.array();

	// Will be used in Update also
	X_abs_w_ = X_abs.array().rowwise() * weights_.transpose().array();

	P_ = X_abs_w_ * X_abs.transpose();
}

/**
* Updates the state and the state covariance matrix using a radar measurement.
* @param {MeasurementPackage} meas_package
*/
void UKF::UpdateRadar(MeasurementPackage meas_package) {
	// Set measurement dimension, radar can measure r, phi, and r_dot
	int n_z = n_radar_;
	// Create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, n_sig_);
	// Transform sigma points into measurement space
	for (int i = 0; i < n_sig_; i++) {
		// extract values for better readibility
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);
		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;
		// Measurement model
		Zsig(0, i) = sqrt(p_x*p_x + p_y * p_y);          //r
		Zsig(1, i) = atan2(p_y, p_x);                   //phi
		Zsig(2, i) = (p_x*v1 + p_y * v2) / Zsig(0, i);   //r_dot
	}
	//std::cout << "Estimation: " << Zsig << endl;
	UpdateUKF(meas_package, Zsig, n_z);
}

/**
* Updates the state and the state covariance matrix using a laser measurement.
* @param {MeasurementPackage} meas_package
*/
void UKF::UpdateLidar(MeasurementPackage meas_package) {
	// Set measurement dimension
	int n_z = n_laser_;
	// Create matrix for sigma points in measurement space
	// Transform sigma points into measurement space
	MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);
	UpdateUKF(meas_package, Zsig, n_z);
}

//  Updates the state and the state covariance matrix
void UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z) {
	// Mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred = Zsig * weights_;
	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {
		// Residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		// Angle normalization
		NormalizeAngle(&(z_diff(1)));
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}
	// Add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z, n_z);
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) { // Radar
		R = R_radar_;
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER) { // Lidar
		R = R_lidar_;
	}
	S = S + R;

	// Create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	// Calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) { // Radar
																	  // Angle normalization
			NormalizeAngle(&(z_diff(1)));
		}
		// State difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		// Angle normalization
		NormalizeAngle(&(x_diff(3)));
		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}
	// Measurements
	VectorXd z = meas_package.raw_measurements_;
	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();
	// Residual
	VectorXd z_diff = z - z_pred;
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) { // Radar
																  // Angle normalization
		NormalizeAngle(&(z_diff(1)));
	}
	// Update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();
}

void UKF::NormalizeAngle(double *ang) {
	while (*ang > M_PI) *ang -= 2. * M_PI;
	while (*ang < -M_PI) *ang += 2. * M_PI;
}