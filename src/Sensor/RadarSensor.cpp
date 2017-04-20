#include "RadarSensor.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
* Constructor.
*/
RadarSensor::RadarSensor()
{
	is_enabled_ = true;
	m_dimensions_ = 3;

	R_ = MatrixXd(m_dimensions_, m_dimensions_);
	// set measurement covariance
	R_ << 0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;
}

/*
* Destructor.
*/
RadarSensor::~RadarSensor() { }

bool RadarSensor::Handles(const MeasurementPackage::SensorType &sensorType)
{
	return (sensorType == MeasurementPackage::SensorType::RADAR);
}

void RadarSensor::Update(const VectorXd &state, const MeasurementPackage &measurement)
{
	state_ = 1.0 * state;
}

/*
* Projects the state into measurement space
* @param state: State to project into measurement space
*/
VectorXd RadarSensor::Project(const VectorXd &state)
{
	VectorXd measurement = VectorXd(m_dimensions_);

	double px = state(0);
	double py = state(1);
	double v = state(2);
	double psi = state(3);
	
	double psqrt = sqrt((px * px) + (py * py));

	// project into polar coordinates
	double rho = psqrt;
	double phi = atan2(py, px);
	double rho_dot = ((px * cos(psi) * v) + (py * sin(psi) * v)) / psqrt;

	measurement << rho, phi, rho_dot;

	return measurement;
}

/*
* Dampens the measurement into state space
* @param measurement: Measurement to project into coordinate space
*/
VectorXd RadarSensor::Dampen(const VectorXd &measurement)
{
	VectorXd state = 1.0 * state_;

	double rho = measurement[0];
	double phi = measurement[1];
	double rho_dot = measurement[2];

	// project into cartesian coordinates
	state(0) = rho * cos(phi);
	state(1) = rho * sin(phi);
	state(2) = rho_dot * cos(phi);
	state(3) = rho_dot * sin(phi);

	return state;
}
