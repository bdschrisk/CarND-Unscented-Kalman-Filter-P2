#ifndef LIDAR_SENSOR_H_
#define LIDAR_SENSOR_H_

#include "../Eigen/Dense"
#include "../measurement_package.h"
#include "Sensor.h"

class LidarSensor : public Sensor
{
public:
	/*
	* Constructor.
	*/
	LidarSensor();
	
	/*
	* Destructor.
	*/
	virtual ~LidarSensor();

	/*
	* Returns True if the sensor type matches the sensor implementation
	*/
	virtual bool Handles(const MeasurementPackage::SensorType &sensorType);

	/*
	* Updates the state of the sensor with the given measurement
	*/
	virtual void Update(const Eigen::VectorXd &state, const MeasurementPackage &measurement);

	/*
	* Projects the state into measurement space
	*/
	virtual Eigen::VectorXd Project(const Eigen::VectorXd &state);

	/*
	* Dampens the measurement into state space
	*/
	virtual Eigen::VectorXd Dampen(const Eigen::VectorXd &measurement);
};

#endif