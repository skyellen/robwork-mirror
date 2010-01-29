
#include <iostream>


//#include <rwhw/can/ESDCAN/ESDCANPort.hpp>

#include <rwhw/PowerCube/Cube.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/common/macros.hpp>
using namespace rwhw;
using namespace rw::common;

/*
CubePort* getCubePortCAN(){
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

    return CubePort::make(canPort);
}
*/
CubePort* getCubePortSerial(const std::string& portname){
	SerialPort *port = new SerialPort();

	if(!port->open(portname, SerialPort::Baud9600))
		RW_THROW("Failed to open serialport!");

	return CubePort::make(port);
}

int main(int argc, char** argv)
{
	std::cout << "***********************************************"<< std::endl;
	std::cout << "* Simple test of the cube interface using either " << std::endl
			  << "* RS232 or CAN for communication " << std::endl;

	if( argc<3 ){

		std::cout << "* Input : " << std::endl
				  << "* 1 arg = [serial,can]" << std::endl
				  << "* 2 arg = [COM1,...] or [ESD]" << std::endl;
		 return 1;
	}
	std::cout << "* " << std::endl
			  << "* INITIALIZING " << std::endl;

	std::string arg1(argv[1]);
	std::string arg2(argv[2]);
	bool useSerial = false;
	CubePort *cport = NULL;
	if( arg1=="serial" ){
		std::cout << "* Using RS232 for communication!" << std::endl;
		std::cout << "* - no broadcast commands available in RS232 mode" << std::endl;
		cport = getCubePortSerial(arg2);
		useSerial = true;
	} else if(arg1=="can"){
		std::cout << "* Using CAN for communication!" << std::endl;
		return 0;//cport = getCubePortCAN();
	} else {
		RW_THROW("Unknown type of communication");
	}

	if( !useSerial ){
		std::cout << "* Resetting all!!" <<std::endl;
		Cube::resetAllCmd(cport);
		TimerUtil::sleepMs(1000);

	    std::cout << "* Homing all!!" <<std::endl;
	    Cube::homeAllCmd(cport);
	    TimerUtil::sleepMs(1000);
	}

    std::cout << "* Checking if any PowerCube devices are connected " << std::endl;
    std::vector<Cube*> cubes = Cube::getCubes(5,13, cport);
    if(cubes.size()>0){
    	std::cout << "* - Found " << cubes.size() << " cubes " << std::endl;
    	std::cout << "* -- Ids: ";
    	for(size_t i=0;i<cubes.size();i++){
    		std::cout << cubes[i]->getCubeID() << " ";
    	}
    	std::cout << std::endl;
    }
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
			cubes[joint]->moveCurCmd(-1.0);
    	} break;
    	case('+'): {
    		//float pos = cubes[joint]->getActPos();
    		cubes[joint]->moveCurCmd(1.0);
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
    std::cout << "* Quit "<< std::endl;
    std::cout << "***********************************************"<< std::endl;

    return 0;
}
