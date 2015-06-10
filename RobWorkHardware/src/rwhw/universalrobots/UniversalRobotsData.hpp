#ifndef RWHW_UNIVERSALROBOTSDATA_HPP
#define RWHW_UNIVERSALROBOTSDATA_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/common/types.hpp>
#include <string>

namespace rwhw {

class UniversalRobotsData {

  public:
	UniversalRobotsData();
  
	//Connection
	std::string errorMessage;

	// timestamp
	double driverTimeStamp;

	long controllerTimeStamp;

	// robot
	int robotMode; // The real robot mode

	// joint
	unsigned char jointMode[6];
	float jointCurrent[6];
	float jointVoltage[6];
	float jointMotorTemperature[6];
	float jointMicroTemperature[6];

	// tool
	float toolVoltage48V;
	float toolTemperature;
	int toolOutputVoltage;
	int toolMode;
	float toolCurrent;

	// master
	float masterTemperature;
	float robotVoltage48V;
	float robotCurrent;
	float masterIOCurrent;

	// q, qd (actual positions and velocities)
	rw::math::Q jointPosition;
	rw::math::Q jointSpeed;
	rw::math::Vector3D<double> toolPosition;
	rw::math::Vector3D<double> toolAxisAngle;

	rw::math::Q targetJointPosition;

	rw::math::Vector3D<double> laserPointerPosition;
	// I/O
	bool digitalIn[10]; // digital input
	bool digitalOut[10]; // digital output
	unsigned char analogInputRange[4];
	unsigned char analogOutputDomain[2];
	double analogIn[4]; // analog input
	double analogOut[2]; // analog output

	// slider speed
	double speedFraction;

	// true = real robot, false = simulated robot
	bool real;

	// true = physical real robot, false = simulated real robot
	bool physical;
	bool robotPowerOn;
	bool emergencyStopped;
	bool securityStopped;
	bool programRunning;
	bool programPaused;

	//script command input
	std::string command;
	bool newCommand;

  private:
};

} // end namespace

#endif //end include guard
