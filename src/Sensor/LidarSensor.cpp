#include "LidarSensor.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
* Constructor.
*/
LidarSensor::LidarSensor()
{
	is_enabled_ = true;
	m_dimensions_ = 2;

	// set measurement covariance
	R_ = MatrixXd(m_dimensions_, m_dimensions_);
	R_ << 0.0225, 0,
				0, 0.0225;
}

/*
* Destructor.
*/
LidarSensor::~LidarSensor() { }

bool LidarSensor::Handles(const MeasurementPackage::SensorType &sensorType)
{
	return (sensorType == MeasurementPackage::SensorType::LASER);
}

void LidarSensor::Update(const VectorXd &state, const MeasurementPackage &measurement)
{
	state_ = 1.0 * state;
}

/*
* Projects the state into measurement space
* @param state: State to project into measurement space
*/
VectorXd LidarSensor::Project(const VectorXd &state)
{
	VectorXd measurement = VectorXd(m_dimensions_);

	double px = state[0];
	double py = state[1];

	measurement << px, py;

	return measurement;
}

/*
* Dampens the measurement into state space
* @param measurement: Measurement to project into coordinate space
*/
VectorXd LidarSensor::Dampen(const VectorXd &measurement)
{
	VectorXd state = 1.0 * state_;

	double px = measurement[0];
	double py = measurement[1];

	state(0) = px;
	state(1) = py;

	return state;
}