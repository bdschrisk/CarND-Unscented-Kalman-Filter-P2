#ifndef SENSOR_H_
#define SENSOR_H_

#include "../Eigen/Dense"
#include "../measurement_package.h"

class Sensor {
public:
	/*
	* Determines whether the sensor is enabled during updates
	*/
	bool is_enabled_;

	/*
	* Gets the measurement dimensions
	*/
	int m_dimensions_;

	/*
	* Gets the state
	*/
	Eigen::VectorXd state_;
	/*
	* Measurement covariance matrix
	*/
	Eigen::MatrixXd R_;
	
	/**
	* Destructor.
	*/
	virtual ~Sensor() { }

	/*
	* Returns True if the sensor type matches the sensor implementation
	*/
	virtual bool Handles(const MeasurementPackage::SensorType &sensorType) = 0;

	/*
	* Updates the state of the sensor with the given measurement
	*/
	virtual void Update(const Eigen::VectorXd &state, const MeasurementPackage &measurement) = 0;

	/*
	* Projects the state into measurement space
	*/
	virtual Eigen::VectorXd Project(const Eigen::VectorXd &state) = 0;

	/*
	* Dampens the measurement into state space
	*/
	virtual Eigen::VectorXd Dampen(const Eigen::VectorXd &measurement) = 0;

};

#endif