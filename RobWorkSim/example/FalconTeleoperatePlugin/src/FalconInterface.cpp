#include "FalconInterface.hpp"

bool FalconInterface::initializeFalcon()
{
	falcon = new FalconDevice();
	
	cout << "Setting up comm interface for Falcon comms" << endl;

	unsigned int count;
	falcon->getDeviceCount(count);
	cout << "Connected Device Count: " << count << endl;

	//Open the device number:
	int deviceNum = 0;
	cout << "Attempting to open Falcon device:  " << deviceNum << endl;
	if(!falcon->open(deviceNum))
	{
		cout << "Cannot open falcon device index " << deviceNum << " - Lib Error Code: " << falcon->getErrorCode() << " Device Error Code: " << falcon->getFalconComm()->getDeviceErrorCode() << endl;
		return false;
	}
	else
	{
		cout << "Connected to Falcon device " << deviceNum << endl ;
	}

	//Load the device firmware:
	//There's only one kind of firmware right now, so automatically set that.
	falcon->setFalconFirmware<FalconFirmwareNovintSDK>();
	falcon->setFalconGrip<FalconGripFourButton>();
	//Next load the firmware to the device
	
	bool skip_checksum = false;
	//See if we have firmware
	bool firmware_loaded = false;
	firmware_loaded = falcon->isFirmwareLoaded();
	if(!firmware_loaded)
	{
		std::cout << "Loading firmware" << std::endl;
		uint8_t* firmware_block;
		long firmware_size;
		{

			firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
			firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


			for(int i = 0; i < 10; ++i)
			{
				if(!falcon->getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))

				{
					cout << "Firmware loading try failed";
					//Completely close and reopen
					//falcon.close();
					//if(!falcon.open(m_varMap["device_index"].as<int>()))
					//{
					//	std::cout << "Cannot open falcon device index " << m_varMap["device_index"].as<int>() << " - Lib Error Code: " << m_falconDevice->getErrorCode() << " Device Error Code: " << m_falconDevice->getFalconComm()->getDeviceErrorCode() << std::endl;
					//	return false;
					//}
				}
				else
				{
					firmware_loaded = true;
					break;
				}
			}
		}
	}
	else if(!firmware_loaded)
	{
		std::cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << std::endl;
		//return false;
	}
	else
	{
		//return true;
	}
	if(!firmware_loaded || !falcon->isFirmwareLoaded())
	{
		std::cout << "No firmware loaded to device, cannot continue" << std::endl;
		//return false;
	}
	std::cout << "Firmware loaded" << std::endl;

	//Seems to be important to run the io loop once to be sure of sensible values next time:
	falcon->runIOLoop();

	falcon->getFalconFirmware()->setHomingMode(false);
	
	falcon->setFalconKinematic<libnifalcon::FalconKinematicStamper>();

	return true;
}

void FalconInterface::run()
{	
	isThreadStopped = false;
	
	while(!isThreadStopped) {
	    unsigned int buttonState = 0;
	    
		falcon->runIOLoop();
		
		//get button values from falcon		
		buttonState = falcon->getFalconGrip()->getDigitalInputs();
		_buttonState = buttonState;
		//get positions from falcon
		pos = falcon->getPosition();

		diff[0] = pos[0];
		diff[1] = pos[1];
		diff[2] = pos[2] - DIFF2_OFFSET;
		
		//get default force values
		defaultForce[0] = DEFAULT_FORCE_COEFFICENT*diff[0];
		defaultForce[1] = DEFAULT_FORCE_COEFFICENT*diff[1];
		defaultForce[2] = DEFAULT_FORCE_COEFFICENT*diff[2];
		
		//arrange transform and force vector
		double sq = sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
		
		if(sq < NO_MOVEMENT_RADIUS) {
			diff[0] = diff[1] = diff[2] = 0;
			strongForce[0] = strongForce[1] = strongForce[2] = 0;
			
			if(sq < NO_FORCE_RADIUS) 
				defaultForce[1] = defaultForce[0] = defaultForce[2] = 0;
				
			isInNMA = true;
		}
		else
		{
			strongForce[0] = diff[0]*(sq - NO_MOVEMENT_RADIUS) / sq;
			strongForce[1] = diff[1]*(sq - NO_MOVEMENT_RADIUS) / sq;
			strongForce[2] = diff[2]*(sq - NO_MOVEMENT_RADIUS) / sq;
		}
		
		
		
		// Add user's force
		if(enableStrongForce) {
			defaultForce[0] += strongForce[0] * STRONG_FORCE_COEFFICENT;
			defaultForce[1] += strongForce[1] * STRONG_FORCE_COEFFICENT;
			defaultForce[2] += strongForce[2] * STRONG_FORCE_COEFFICENT;
		}
		
		//apply gravity
		defaultForce[1] += GRAVITY_FORCE;
		
		//apply force
		falcon->setForce(defaultForce);
		
		//decide rotation or translation by pressing buttons
		if (buttonState == 4)
		{
			//decrease transform values for slower rotation
			diff[0] /=  ROTATE_COEFFICIENT;
			diff[1] /= -ROTATE_COEFFICIENT;
			diff[2] /= -ROTATE_COEFFICIENT;
			
			position = Transform3D<>( Vector3D<>(0,0,0) , EAA<>( Vector3D<>(-diff[1], diff[0], 0 )));
		}
			
		else if (buttonState == 2)
		{
			//decrease transform values for slower rotation
			diff[0] /=  ROTATE_COEFFICIENT;
			diff[1] /= -ROTATE_COEFFICIENT;
			diff[2] /= -ROTATE_COEFFICIENT;
			
			position = Transform3D<>( Vector3D<>(0,0,0) , EAA<>( Vector3D<>(0,0,diff[0])));
		}
		
		else
		{
			//decrease transform values for slower movement
			diff[0] /=  MOVEMENT_COEFFICIENT;
			diff[1] /= -MOVEMENT_COEFFICIENT;
			diff[2] /= -MOVEMENT_COEFFICIENT;
			
			position = Transform3D<>( diff, RPY<>(0,0,0) );
		}
			
		//grasping
		if (buttonState == 8 )
			grasping = true;
		if (buttonState == 1)
			grasping = false;
			
		usleep(10);
	}
}

void FalconInterface::setStrongForce(bool _enableStrongForce) {

	enableStrongForce = _enableStrongForce;
	
}

unsigned int FalconInterface::getButtonStates() {

    return _buttonState;
}


bool FalconInterface::getGrasping() {

	return grasping;
}

Transform3D<> FalconInterface::getPosition() {

	return position;
}
