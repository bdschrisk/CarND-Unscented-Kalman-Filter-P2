#include "ukf.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

	// set dimensions
	n_x_ = 5;
	n_aug_ = 7;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.808;
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.707;

	// define spreading factor
	lambda_ = (0.5 * (n_x_ + n_aug_) - (n_aug_ + n_aug_)) / 2.0;
	// max time delta between predictions
	time_delta = 0.1;

	// initial initialisation...
	is_initialized_ = false;

	// initial state vector
	x_ = VectorXd::Random(n_x_);

	// initial covariance matrix
	P_ = MatrixXd::Identity(n_x_, n_x_);

	// initialise process noise covariance matrix
	Q_ = MatrixXd(n_aug_ - n_x_, n_aug_ - n_x_);
	Q_ << std_a_ * std_a_, 0,
				0, std_yawdd_ * std_yawdd_;

	radarSensor_.is_enabled_ = use_radar_;
	lidarSensor_.is_enabled_ = use_laser_;

	sensors_.push_back(&radarSensor_);
	sensors_.push_back(&lidarSensor_);

	// Initialise weights
	weights_ = VectorXd(2 * n_aug_ + 1);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		if (i == 0) {
			weights_(i) = lambda_ / (lambda_ + n_aug_);
		}
		else {
			weights_(i) = 1.0 / (2.0 * (lambda_ + n_aug_));
		}
	}
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement) {
	// Initialise Sensor
	Sensor* sensor;

	size_t N = sensors_.size();
	for (size_t i = 0; i < N; i++) {
		if (sensors_[i]->Handles(measurement.sensor_type_)) {
			sensor = sensors_[i];
			break;
		}
	}

	// check valid sensor
	if (sensor == NULL) {
		cout << "ProcessMeasurement() - Error: Unknown sensor type '" << measurement.sensor_type_ << "'";
		return;
	}

	NIS_ = 0;

	// update sensor with new measurements
	sensor->Update(x_, measurement);

	if (!is_initialized_) {
		// first measurement
		cout << "UKF: " << endl;

		// initialise state at t0 using positions only
		x_ = sensor->Dampen(measurement.raw_measurements_);

		// store previous timestamp
		previous_timestamp_ = measurement.timestamp_;

		// done initializing, no need to predict or update
		is_initialized_ = true;

		return;
	}

	// compute the time delta
	double dt = (measurement.timestamp_ - previous_timestamp_) / 1000000.0;	// in seconds
	previous_timestamp_ = measurement.timestamp_;

	/*****************************************************************************
	*  UKF Predict and Update
	****************************************************************************/
	// Predict Sigma points for transition

	// check for multiple simultaneous measurements
	while (dt > time_delta) {
		this->Sigma(time_delta);
		this->Predict();

		dt -= time_delta;
	}

	this->Sigma(dt);
	// Predicts state P
	this->Predict();

	if (sensor->is_enabled_) {
		VectorXd pos = sensor->Dampen(measurement.raw_measurements_);

		// check numerical stability
		if ((pos(0) > FLT_EPSILON || pos(0) < -FLT_EPSILON)
			&& (pos(1) > FLT_EPSILON || pos(1) < -FLT_EPSILON)) {
			// Update

			// measurement
			VectorXd z = measurement.raw_measurements_;

			// apply measurement transform
			MatrixXd Zsigma = this->Transform(sensor);

			// update state with observed measurements
			this->Update(Zsigma, z, sensor->R_);
		}
	}

	// print the output
	cout << "x_ = " << x_ << endl;
	cout << "P_ = " << P_ << endl;
}

/*
* Applies the sigma points of the current timestep using augmentation
* @param delta_t: Time delta expressed in seconds
*/
void UKF::Sigma(double delta_t) {

	// initialise process noise covariance matrix
	MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q_;

	VectorXd x_aug = VectorXd::Zero(n_aug_);

	// combine state with process noise
	x_aug.head(n_x_) = x_;
	
	MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
	// calculate square root of P(augmented)
	MatrixXd A = P_aug.llt().matrixL();
	
	// set first augmented state as current state
	Xsig_aug.col(0) = x_aug;

	// create augmented sigma points
	for (int i = 0; i < n_aug_; i++) {
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
		Xsig_aug.col(i + n_aug_ + 1) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
	}

	double d2 = delta_t * delta_t;

	// create X'
	Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

	//predict sigma points at t+1 using augmented states
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {

		double px = Xsig_aug(0, i);
		double py = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double psi = Xsig_aug(3, i);
		double psi_dot = Xsig_aug(4, i);
		double mu_a = Xsig_aug(5, i);
		double mu_psi_dot = Xsig_aug(6, i);

		double px_mu = 0.5 * mu_a * d2 * cos(psi);
		double py_mu = 0.5 * mu_a * d2 * sin(psi);
		double v_mu = delta_t * mu_a;
		double psi_mu = 0.5 * d2 * mu_psi_dot;
		double psi_dot_mu = delta_t * mu_psi_dot;

		double px_p = 0;
		double py_p = 0;
		double v_p = 0;
		double psi_p = psi_dot * delta_t;
		double psi_dot_p = 0;

		if (fabs(psi_dot) > 0.001) {
			px_p = (v / psi_dot) * (sin(psi + psi_dot*delta_t) - sin(psi));
			py_p = (v / psi_dot) * (-cos(psi + psi_dot*delta_t) + cos(psi));
		}
		else {
			px_p = v * delta_t * cos(psi);
			py_p = v * delta_t * sin(psi);
		}

		Xsig_pred_(0, i) = px + px_p + px_mu;
		Xsig_pred_(1, i) = py + py_p + py_mu;
		Xsig_pred_(2, i) = v + v_p + v_mu;
		Xsig_pred_(3, i) = psi + psi_p + psi_mu;
		Xsig_pred_(4, i) = psi_dot + psi_dot_p + psi_dot_mu;
	}
}

/*
* Transforms the predicted sigma points into measurement space
*/
MatrixXd UKF::Transform(Sensor* sensor) {
	// initialise measurement sigma
	MatrixXd Zsigma = MatrixXd(sensor->m_dimensions_, 2 * n_aug_ + 1);

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd x = Xsig_pred_.col(i);
		VectorXd z = sensor->Project(x);

		Zsigma.col(i) = z;
	}

	return Zsigma;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Predict() {
	
	x_.fill(0.0);
	// predict state mean
	for (int i = 0; i < n_x_; i++) {
		double sum = 0;

		for (int j = 0; j < 2 * n_aug_ + 1; j++) {
			sum += weights_(j) * Xsig_pred_(i, j);
		}

		x_(i) = sum;
	}

	P_.fill(0.0);
	// predict state covariance matrix
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;

		P_ = P_ + weights_(i) * (x_diff * x_diff.transpose());
	}
}

/**
* Updates the state and the state covariance matrix using a the given sensor measurement
* @param z: Measurement vector at t+1
* @param z_pred: Predicted measurement at t+1
* @param R: Sensor measurement covariance matrix
*/
void UKF::Update(const MatrixXd &Zsigma, const VectorXd &z, const MatrixXd &R) {
	
	// set measurement dimensions
	int n_z = Zsigma.rows();

	VectorXd z_pred = VectorXd::Zero(n_z);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_pred = z_pred + weights_(i) * Zsigma.col(i);
	}

	// create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

	// calculate cross correlation matrix
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		VectorXd z_diff = Zsigma.col(i) - z_pred;

		Tc = Tc + weights_(i) * (x_diff * z_diff.transpose());
	}

	//measurement covariance matrix S
	MatrixXd S = MatrixXd::Zero(n_z, n_z);
	//calculate measurement covariance matrix S
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd z_diff = (Zsigma.col(i) - z_pred);
		S = S + weights_(i) * (z_diff * z_diff.transpose());
	}

	S = S + R;

	MatrixXd Si = S.inverse();

	// calculate Kalman gain K;
	MatrixXd K = Tc * Si;

	VectorXd z_dp = (z - z_pred);

	// update state mean and covariance matrix
	x_ = x_ + K * z_dp;
	P_ = P_ - K * S * K.transpose();

	// update NIS value
	VectorXd z_d = (z_pred - z);
	NIS_ = z_d.transpose() * Si * z_d;
}
