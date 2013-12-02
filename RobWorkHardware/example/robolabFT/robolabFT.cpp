
#include <iostream>

//#include <rwhw/can/ESDCAN/ESDCANPort.hpp>
//#include <rwhw/serialport/SerialPort.hpp>
#include <rwhw/robolabFT/RobolabFTDriver.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Timer.hpp>
#include <rw/common/macros.hpp>

#include <rw/math/Q.hpp>

using namespace rw::math;
using namespace rw::common;

using namespace rwhw;
using namespace rw::common;

int main(int argc, char** argv)
{
	RobolabFT* ftSensor = new RobolabFT();
    if(ftSensor->init("/dev/rfcomm0",SerialPort::Baud115200,1))
    	ftSensor->run();

	std::cout << "***********************************************"<< std::endl;
	std::cout << "* Simple test of the RobotlabFT driver " << std::endl;
	//std::cout <<ftdata.data.first <<std::endl;

/*
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

