#include "UniversalRobotsData.hpp"

using namespace rwhw;

UniversalRobotsData::UniversalRobotsData()
{
    // timestamp
	driverTimeStamp = 0;
	controllerTimeStamp = 0;

	// joint
	for(unsigned int i=0; i<6; i++)
	{
		jointMode[i]=0;
		jointCurrent[i]=0;
		jointVoltage[i]=0;
		jointMotorTemperature[i]=0;
		jointMicroTemperature[i]=0;
	}

	// tool
	toolVoltage48V=0;
	toolTemperature=0;
	toolOutputVoltage=0;
	toolMode=0;
	toolCurrent=0;

	// master
	masterTemperature=0;
	robotVoltage48V=0;
	robotCurrent=0;
	masterIOCurrent=0;

	rw::math::Vector3D<double> tmp(0,0,0);
	// q, qd (actual positions and velocities)
	jointPosition = rw::math::Q::zero(6);
	jointSpeed = rw::math::Q::zero(6);
	toolPosition = tmp;
	toolAxisAngle = tmp;
	targetJointPosition = rw::math::Q::zero(6);
	laserPointerPosition = tmp;

	// I/O
	for(unsigned int i=0; i<10; i++)
	{
		digitalIn[i]=0; // digital input
		digitalOut[i]=0;// digital output
	}
	for(unsigned int i=0; i<4; i++)
	{
		analogInputRange[i]=0;
		analogIn[i]=0;// analog input
	}
	for(unsigned int i=0; i<2; i++)
	{
		analogOutputDomain[i]=0;
		analogOut[i]=0;// analog output
	}

	// slider speed
	speedFraction = 1.0;

	// true = real robot, false = simulated robot
	 real = false;

	// true = physical real robot, false = simulated real robot
	 physical = false;
	 robotPowerOn = false;
	 emergencyStopped = false;
	 securityStopped = false;
	 programRunning = false;
	 programPaused = false;

	//script command input
	command="";
	newCommand = false;
}

