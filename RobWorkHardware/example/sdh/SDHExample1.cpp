
#include <iostream>

//#include <rwhw/can/ESDCAN/ESDCANPort.hpp>
//#include <rwhw/serialport/SerialPort.hpp>
#include <rwhw/sdh/SDHDriver.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Timer.hpp>
#include <rw/common/macros.hpp>

#include <rw/math/Q.hpp>

using namespace rw::math;
using namespace rw::common;

using namespace rwhw;
using namespace rw::common;

#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

char const* __help__      = "Move proximal and distal joints of finger 1 three times by 10 degrees, stop movement when halfway done.\n(C++ demo application using the SDHLibrary-CPP library.)";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-simple2.cpp 3686 2008-10-13 15:07:24Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";


USING_NAMESPACE_SDH

/*
CanPort* getCAN(){
	std::cout << "Looking for connected can devices" << std::endl;
	std::vector<ESDCANPort::CanDeviceStatus> canDevices =
        ESDCANPort::getConnectedDevices();

	std::cout << "Getting the first connected CAN device" << std::endl;
    //static ESDCANPort* getPortInstance(
    //    unsigned int netId, long txQueueSize=1, long rxQueueSize=1,
	//	CanBaud canBaud=CanBaud250, int transmitDelay=0); // TODO: add baud ad can id type
	ESDCANPort *canPort = ESDCANPort::getPortInstance(0,1,1,ESDCANPort::CanBaud1000);

    if( canPort==NULL ){
        std::cout << "No canPort available." << std::endl;
        return 0;
    }

    std::cout << "Openning the canport" << std::endl;
    canPort->open();

    if(!canPort->isOpen()){
        std::cout << "Canport not open!!" << std::endl;
        return 0;
    }

    return canPort;
}

SerialPort* getSerial(const std::string& portname){
	SerialPort *port = new SerialPort();

	if(!port->open(portname, SerialPort::Baud9600))
		RW_THROW("Failed to open serialport!");

	return port;
}
*/
int main(int argc, char** argv)
{
	std::cout << "***********************************************"<< std::endl;
	std::cout << "* Simple test of the SDH interface using " << std::endl
			  << "* RS232 for sensors and CAN for motion communication " << std::endl;

    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options;

    options.Parse( argc, argv, __help__, "demo-simple2", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );

    cDBG cdbg( options.debug_level > 0, "red", options.debuglog );
    g_sdh_debug_log = options.debuglog;

    cdbg << "Debug messages of " << argv[0] << " are printed like this.\n";


    /*

	if( argc<3 ){
		std::cout << "* Input : " << std::endl
				  << "* 1 arg = [COM1,...]" << std::endl
				  << "* 2 arg = [ESD] " << std::endl;
		 return 1;
	}

	std::cout << "* " << std::endl
			  << "* INITIALIZING " << std::endl;

	std::string arg1(argv[1]);
	std::string arg2(argv[2]);
	bool useSerial = false;
*/
	//CanPort *cport = getCAN();

	//canClose(((ESDCANPort*)cport)->getHandle());

	std::cout << "construct hand" << std::endl;

    // Create an instance "hand" of the class cSDH:
    //cSDH hand( true, false, 4 );
/*
	cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );
    std::cout << "openning hand" << std::endl;
    int net = 0;
    unsigned long baud = 1000000;
    double timeout = -1.0;
    unsigned int id_read = 43;
    unsigned int id_write = 42;

    hand.OpenCAN_ESD( net,          // ESD CAN net number
					  baud, // CAN baudrate in bit/s
					  timeout,
					  id_read,
					  id_write);      // timeout in s


    std::cout << options.net << std::endl
			  << options.can_baudrate << std::endl
			  << options.timeout << std::endl
			  << options.id_read << std::endl
			  << options.id_write << std::endl;

    if ( options.use_can )
        hand.OpenCAN_ESD( options.net,          // ESD CAN net number
                          options.can_baudrate, // CAN baudrate in bit/s
                          options.timeout,      // timeout in s
                          options.id_read,      // CAN ID used for reading
                          options.id_write );   // CAN ID used for writing

    cSDH::eVelocityProfile old_profile = hand.GetVelocityProfile();
    hand.SetVelocityProfile( cSDH::eVP_RAMP );

*/


	std::cout << "* creating SDH " << std::endl;
	SDHDriver *sdh = new SDHDriver();
	std::cout << "* connecting SDH " << std::endl;
	sdh->connect( 0, 1000000 );
	if(!sdh->isConnected())
		RW_THROW("SDH could not connect to hardware!");
	std::cout << "* sdh connected! " << std::endl
			  << "*" << std::endl;

	std::pair<Q,Q> posL = sdh->getPosLimits();
	Q velL = sdh->getVelLimits();
	Q currL = sdh->getCurrentLimits();
	Q target = sdh->getTargetQ();
	Q pos = sdh->getQ();

	std::cout << "* Initial configuration: " << std::endl
			  << "* low pos limit: " << posL.first << std::endl
			  << "* upp pos limit: " << posL.second << std::endl
			  << "* vel limit    : " << velL << std::endl
			  << "* current limit: " << currL << std::endl
			  << "* Target       : " << target << std::endl
			  << "* pos          : " << pos << std::endl;

	std::cout << "*" << std::endl;
	Q zeroQ(Q::zero(pos.size()));

	std::cout << "* Performing blocking move command to:" << std::endl
			  << "* " << zeroQ << std::endl;
	sdh->moveCmd(zeroQ, true);

	std::cout << "Sleeping..." << std::endl;
	TimerUtil::sleepMs(4000);

	std::cout << "* Performing non-blocking move command to:" << std::endl
			  << "* " << pos << std::endl;
	sdh->moveCmd(pos, false);
	Timer time;
	while(!sdh->waitCmd(0.02)){
		Q cpos = sdh->getQ();
		std::cout << "Pos: [" << time.getTime() << "s] - " << cpos << std::endl;
	}

/*
	std::cout << "***********************************************"<< std::endl;
    std::cout << "* Choose cube from keys [1-9]. " << std::endl
			  << "* '-' or '+' moves the joint in current mode." << std::endl
			  << "* 'r' resets the cube. 's' halts the cube." << std::endl
			  << "* 'h' homes the cube. " << std::endl;
    std::cout << "*" << std::endl;
	int joint = 0;
	size_t selectedJoint=1;
	bool running = true;
    while(running){
    	std::cout << "* : ";
    	char key;
    	std::cin >> key;
    	switch(key){
    	case('1'): selectedJoint = 1; break;
    	case('2'): selectedJoint = 2; break;
    	case('3'): selectedJoint = 3; break;
    	case('4'): selectedJoint = 4; break;
    	case('5'): selectedJoint = 5; break;
    	case('6'): selectedJoint = 6; break;
    	case('7'): selectedJoint = 7; break;
    	case('8'): selectedJoint = 8; break;
    	case('9'): selectedJoint = 9; break;
    	case('s'): cubes[joint]->haltCmd(); break;
    	case('r'): cubes[joint]->resetCmd(); break;
    	case('h'): cubes[joint]->homeCmd(); break;
    	case('-'):{
    		//float pos = cubes[joint]->getActPos();
			cubes[joint]->moveCurCmd(-1.1);
    	} break;
    	case('+'): {
    		//float pos = cubes[joint]->getActPos();
    		cubes[joint]->moveCurCmd(1.1);
    	} break;

    	case('q'): {
    		running = false;
    	} break;
     	default:
    		std::cout << " -- Unknown key!'";
    	}
    	if(joint!= ((int)selectedJoint)-1){
			if(selectedJoint < (cubes.size()+1)){
				joint = selectedJoint-1;
				std::cout << " Cube: " << cubes[joint]->getCubeID() << " selected!";
			}
    	}
    	std::cout << std::endl;
    }
    */
    std::cout << "* Quit "<< std::endl;
    std::cout << "***********************************************"<< std::endl;

    return 0;
}
