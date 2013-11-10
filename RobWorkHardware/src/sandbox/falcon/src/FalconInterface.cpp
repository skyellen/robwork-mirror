/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/
 
#include "FalconInterface.hpp"
 
#include <iostream>
#include <boost/function.hpp>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/grip/FalconGripFourButton.h>
#include <falcon/util/FalconCLIBase.h>
#include <falcon/util/FalconFirmwareBinaryNvent.h>
#include <falcon/kinematic/stamper/StamperUtils.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/core/FalconGeometry.h>
#include <falcon/gmtl/gmtl.h>
 
using namespace std;
using namespace rwhw;
using namespace rw::math;
using namespace libnifalcon;



FalconInterface::FalconInterface() :
	_falcon(NULL),
	_thread(NULL),
	_centerPosition(Vector3D<>(0, 0, 0))
{
	_position[0] = _position[1] = _position[2] = 0.0;
	_force[0] = _force[1] = _force[2] = 0.0;
	
	initialize();
	setCenteringMode(false);
}



FalconInterface::~FalconInterface()
{
}



bool FalconInterface::initialize()
{
	/* This function contains code copied from Baris & Batu's wrapper, without major changes */
	
	_falcon = new FalconDevice();
	
	cout << "Setting up comm interface for Falcon comms" << endl;

	unsigned int count;
	_falcon->getDeviceCount(count);
	cout << "Connected Device Count: " << count << endl;

	//Open the device number:
	int deviceNum = 0;
	cout << "Attempting to open Falcon device:  " << deviceNum << endl;
	if (!_falcon->open(deviceNum)) {
		cout << "Cannot open falcon device index " << deviceNum << " - Lib Error Code: " << _falcon->getErrorCode() << " Device Error Code: " << _falcon->getFalconComm()->getDeviceErrorCode() << endl;
		return false;
	} else {
		cout << "Connected to Falcon device " << deviceNum << endl ;
	}

	//Load the device firmware:
	//There's only one kind of firmware right now, so automatically set that.
	_falcon->setFalconFirmware<FalconFirmwareNovintSDK>();
	_falcon->setFalconGrip<FalconGripFourButton>();
	//Next load the firmware to the device
	
	bool skip_checksum = false;
	//See if we have firmware
	bool firmware_loaded = false;
	firmware_loaded = _falcon->isFirmwareLoaded();
	if (!firmware_loaded) {
		std::cout << "Loading firmware" << std::endl;
		uint8_t* firmware_block;
		long firmware_size;
		{

			firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
			firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;

			for (int i = 0; i < 10; ++i) {
				if (!_falcon->getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE))) {
					cout << "Firmware loading try failed";
				} else {
					firmware_loaded = true;
					break;
				}
			}
		}
	} else if (!firmware_loaded) {
		std::cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << std::endl;
		//return false;
	} else {
		//return true;
	}
	
	if (!firmware_loaded || !_falcon->isFirmwareLoaded()) {
		std::cout << "No firmware loaded to device, cannot continue" << std::endl;
		//return false;
	}
	std::cout << "Firmware loaded" << std::endl;

	//Seems to be important to run the io loop once to be sure of sensible values next time:
	_falcon->runIOLoop();

	_falcon->getFalconFirmware()->setHomingMode(false);
	
	_falcon->setFalconKinematic<libnifalcon::FalconKinematicStamper>();
	
	return true;
}



void FalconInterface::start()
{
	cout << "Starting Falcon thread..." << endl;
	if (!_thread) {
		_running = true;
		_thread = new boost::thread(boost::bind(&FalconInterface::falconLoop, this));
	}
}



void FalconInterface::stop()
{
	_running = false;
	
	_thread->join();
	
	_thread = NULL;
}



rw::math::Vector3D<> FalconInterface::getPosition() const
{
	boost::mutex::scoped_lock scoped_lock(_mutex);
	Vector3D<> pos(_position[0], _position[1], _position[2]);
	
	return pos;
}



unsigned int FalconInterface::getButtonState() const
{
	boost::mutex::scoped_lock scoped_lock(_mutex);
	unsigned int state = _buttonState;
	
	return state;
}



void FalconInterface::setForce(const rw::math::Vector3D<>& force)
{
	boost::mutex::scoped_lock scoped_lock(_mutex);
	_force[0] = force(0);
	_force[1] = force(1);
	_force[2] = force(2);
}



rw::math::Vector3D<> FalconInterface::getForce() const
{
	boost::mutex::scoped_lock scoped_lock(_mutex);
		
	return Vector3D<>(_force[0], _force[1], _force[2]);
}



void FalconInterface::setCenteringMode(bool enable, double forceCoeff, double deadRadius)
{
	boost::mutex::scoped_lock scoped_lock(_mutex);

	_centeringMode = enable;
	_centeringForceCoeff = forceCoeff;
	_centeringDeadZoneRadius = deadRadius;
}



void FalconInterface::falconLoop()
{
	while (_running) {
		_falcon->runIOLoop();
		
		{
			boost::mutex::scoped_lock scoped_lock(_mutex);
			_position = _falcon->getPosition();
			_position[2] += zAxisOffset;
			
			_buttonState = _falcon->getFalconGrip()->getDigitalInputs();
			
			// compute additional force for the centering mode
			if (_centeringMode) {
				Vector3D<> pos(_position[0], _position[1], _position[2]);
				Vector3D<> cForce = _centerPosition - pos;
				
				if (cForce.norm2() < _centeringDeadZoneRadius) {
					cForce = Vector3D<>::zero();
				} else {
					cForce *= _centeringForceCoeff;
				}
				
				_force[0] = cForce[0];
				_force[1] = cForce[1] + gravityForce;
				_force[2] = cForce[2];
			}
			
			_falcon->setForce(_force);
		}
		
		boost::thread::yield();
	}
}
