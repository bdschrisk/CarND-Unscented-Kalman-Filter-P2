# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---

## Overview
In this project an Unscented Kalman Filter has been employed to accurately track a moving pedestrian using sensor fusion.  Sensor data from Lidar and Radar
sensors are jointly utilised to update the belief state of the UKF.  Internally the UKF maintains a probability distribution using a 
Constant Turn Rate and Velocity magnitude model, this allows accurate tracking of objects with non-linear movements - in this case a pedestrian.

In the CTRV model, the UKF maintains a state with not only the cartesian coordinates, but the velocity, yaw angle, as well as the accompanying yaw change rate.
This allows the UKF to accurately track position, velocity and heading, which is highly desirable in noisy environments such as traffic scenarios where objects
don't always move in straight lines.

In comparison to an Extended Kalman Filter, the UKF no longer requires taylor expansion within the measurement update step.  Instead the UKF maintains a probability
distribution using a pseudo point cloud called sigma points, centered around the mean, to narrow down the update possibilities.  The new update rule is 
determined by these points using the state mean and corresponding probability distribution, and calculated by a weighted sum of the spread of these points.

The UKF makes a trade-off between complexity and motion accuracy, whereas the EKF is limited in its linear motion tracking, the UKF solves this using the 
abovementioned process model (CTRV) for tracking non-linear movements.

## Structure
The Unscented Kalman Filter algorithm has been decoupled in such a way that will allow it to seamlessly handle linear and non-linear measurements.  This is 
performed using invariant projection and dampening methods in the corresponding sensor class. The projections are defined for converting between cartesian space, 
required by the kalman filter, and the sensor measurement space.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.
