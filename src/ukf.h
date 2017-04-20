#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include "measurement_package.h"
#include "tools.h"
#include "Sensor/RadarSensor.h"
#include "Sensor/LidarSensor.h"

class UKF {
private:
	RadarSensor radarSensor_;
	LidarSensor lidarSensor_;

public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  ///* state covariance matrix
	Eigen::MatrixXd P_;

	///* Process noise covariance matrix
	Eigen::MatrixXd Q_;

  ///* predicted sigma points matrix
	Eigen::MatrixXd Xsig_pred_;

	///* Previous timestamp
	long long previous_timestamp_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Weights of sigma points
	Eigen::VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

	///* Current Normalised Innovation Squared value
	double NIS_;

	///* Minimum delay between sensor measurements
	double time_delta;

	///* Sensors array
	std::vector<Sensor*> sensors_;

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
  * @param measurement: The latest sensor measurement data
  */
  void ProcessMeasurement(MeasurementPackage measurement);

	/*
	* Sets the time delta of the transition state function
	* @param delta_t: Time delta expressed in seconds
	*/
	void Sigma(double delta_t);
  
	/*
	* Transforms the sigma point into measurement space
	*/
	Eigen::MatrixXd Transform(Sensor* sensor);

	/**
  * Predicts sigma points, the state, and the state covariance matrix
  */
  void Predict();

  /**
  * Updates the state and the state covariance matrix using the given sensor measurement
  * @param z: Measurement vector at t+1
	* @param z_pred: Predicted measurement at t+1
	* @param R: Sensor measurement covariance matrix
  */
	void Update(const Eigen::MatrixXd &Zsigma, const Eigen::VectorXd &z, const Eigen::MatrixXd &R);
};

#endif /* UKF_H */
