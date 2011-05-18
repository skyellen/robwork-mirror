#include "PG70Controller.hpp"

// STL
//#include <unistd.h>
#include <sys/stat.h>
#include <cmath>
// RW
#include <rw/common/Exception.hpp>
#include <rw/common/TimerUtil.hpp>


const float PG70Controller::HOMEPOS = 0.034f;
const float PG70Controller::VEL = 0.07f;
const float PG70Controller::ACC = 0.035f;
const float PG70Controller::MAXPOS = 0.068f;
const float PG70Controller::MAXVEL = 0.095f;
const float PG70Controller::MAXACC = 0.039f;
const float PG70Controller::MAXCUR = 8.5f;

using namespace rwhw;
// Constructor
PG70Controller::PG70Controller() : 
_defMinPos(0), _defMaxPos(0), _defMaxDeltaVel(0),
_defTorqueRatio(0), _defCurRatio(0),
_defMinVel(0), _defMaxVel(0), _defMinAcc(0), _defMaxAcc(0),
_defMinCur(0), _defMacCur(0)
{
	_connected = false;
	_cubePort = NULL;
	_cube = NULL;
	_port = NULL;
}

// Destructor
PG70Controller::~PG70Controller() {
	delete _port;
	if(_cubePort)
		delete _cubePort;
	if(_cube)
		delete _cube;
}

bool PG70Controller::connect(const std::string& port) {
	if(initialize(port)) {
		unsigned int statusMem = 0;
		bool getStatusMem = false;
		bool dataComOK = false;
		_connected = true;

		goHome();
			
		//Get min and max limits
		bool tmp = true;
		do {
			try {
				_port->clean();
				_defMaxPos = _cube->getDefMinPos();
				_port->clean();
				_defMinPos = _cube->getDefMaxPos();
				float maxcur = _cube->getMaxCur();
				std::cout<<"MAXIMAL CURRENT = "<<maxcur<<std::endl;
				tmp=false;
			} catch(rw::common::Exception& e) {
				rw::common::TimerUtil::sleepMs(100);
				tmp=true;
				printf("faild to get Position limits");
			}
		}while(tmp);

		printf("Position limits: %f to %f",_defMinPos, _defMaxPos);

		setGraspPowerPct(5.0);
		logTextReadySig("Parallel gripper ready");
		return true;
	} else {
		logTextReadySig("Failed to connect to parallel gripper", true);
		return false;
	}
}

void PG70Controller::disconnect() {
	if (isConnected()) {
		_port->close();
		delete _port;
		_port = NULL;
		_cube = NULL;
		_cubePort = NULL;
	}
}

// Connect to gripper
bool PG70Controller::initialize(const std::string& port) {
	// Find the right COM port
	/*std::vector<std::string> comports;
	comports.push_back("/dev/ttyUSB0");
	comports.push_back("/dev/ttyUSB1");
	comports.push_back("/dev/ttyUSB2");*/
	// Attempt to open the serial port
	if (_port != NULL)
		delete _port;

	_port = new rwhw::SerialPort();
	bool success;
	struct stat fileInfo;
	if(stat(port.c_str(), &fileInfo) == 0) { // The port exists
		success = _port->open(port, rwhw::SerialPort::Baud9600);
	} else {
		logTextReadySig("The port does not exist!", true);
		return false;
	}


	if(!success) {
		delete _port;
		_port = NULL;
		logTextReadySig("Parallel gripper not found", true);
		return false;
	}

	// Make a cube port
	std::vector<rwhw::Cube*> cubes;
	try {
		_cubePort = CubePort::make(_port);
		cubes = rwhw::Cube::getCubes(12, 13, _cubePort);
	} catch(rw::common::Exception& e) {
		logTextReadySig("Parallel gripper error: "+e.what(), true);
		return false;
	}
	if(cubes.size() == 0) {
		logTextReadySig("Parallel gripper error: no cubes found", true);
		return false;
	}

	_cube = cubes.front();
	_port->clean();
	if(_cube==NULL){
		logTextReadySig("Parallel gripper error: no cubes found", true);
		return false;
	}

	if(!rwhw::Cube::ping(_cube->getCubeID(),_cubePort)){
		logTextReadySig("Parallel gripper is not connected", true);
		return false;
	}

	logTextReadySig("Parallel gripper is connected", false);

	return true;
}

// Apply a constant force proportional to the chosen grasp current
void PG70Controller::applyGrasp() {

	try {
		_port->clean();
		while(!_cube->resetCmd()) { rw::common::TimerUtil::sleepMs(100); _port->clean();}
		_port->clean();

		/*unsigned int statusMem;		
			
			status(statusMem);
			if(statusMem & STATE_POW_INTEGRALERR) {
				logTextReadySig("Motor continuous load limit exceeded", true);				
			}
			if((statusMem & (STATE_BEYOND_SOFT | STATE_BEYOND_HARD))>0) {
				logTextReadySig("Position limits exceeded", true);
			}
			if ((statusMem & STATE_CURLIMIT)) {
				logTextReadySig("Current Limit", true);
			}	*/
		//_cube->haltCmd();
		//_cube->resetCmd();
			//_port->clean();
		while(!_cube->moveCurCmd(_graspCurrent)) { rw::common::TimerUtil::sleepMs(100); _port->clean();}
		
	} catch(rw::common::Exception& e) {
		std::cout<< "Parallel gripper error: " << e.what() << std::endl;
	}
}

// Set force to zero
void PG70Controller::stopGrasp() {
	_port->clean();
	try {
		//_cube->resetCmd();
		_cube->moveCurCmd(0.0f);
	} catch(rw::common::Exception& e) {
		std::cout << "Parallel gripper error: " << e.what() << std::endl;
	}
}

// Stop grasp by retracting grippers
void PG70Controller::retractGrippers() {
	//_port->clean();
	//_cube->moveCurCmd(-_graspCurrent);
	// Set desired position 2 cm backwards
	rw::math::Q q(1);
	
	getQ(q);
	q[0] += 0.02;
	// Get position limit
	rw::math::Q qmax(1);
	qmax[0] = MAXPOS/2.0;
	setQ(qmax);
	// Handle limit when setting position
	//q[0] > MAXPOS ? setQ(qmax) : setQ(q);
}

// Send gripper to home position
void PG70Controller::goHome() {
	try {
		_port->clean();
		while(!_cube->resetCmd()) { rw::common::TimerUtil::sleepMs(100); _port->clean();}
		while(!_cube->homeCmd()) { rw::common::TimerUtil::sleepMs(100); _port->clean();}
		unsigned int statusMem=0;
		bool getStatusMem=false;
		do{
			rw::common::TimerUtil::sleepMs(10);
			getStatusMem = status(statusMem);
		}while(!(statusMem & STATE_HOME_OK) || (!getStatusMem));
		logTextReadySig("Home is found successful", true);
	} catch(rw::common::Exception& e) {
		std::cout << "Parallel gripper error: " << e.what() << std::endl;
	}
	//  rw::math::Q qHome(1);
	//  qHome[0] = HOMEPOS;
	//  setQ(qHome);
}

// Get gripper configuration
bool PG70Controller::getQ(rw::math::Q &q) {
	_port->clean();

	try {
		double tmp = 0.5 * (double)_cube->getActPos();
		q(0)=tmp;
	} catch(rw::common::Exception& e) {
		std::cout << "Parallel gripper error: " << e.what() << std::endl;
		return false;
	}

	return true;
}

// Set gripper configuration
bool PG70Controller::setQ(const rw::math::Q& q) {
	if(q.size() < 1) {
		logTextReadySig("Invalid configuration", true);
		return false;
	}
	rw::math::Q qTmp = 2.0*q;
	if(qTmp[0] < 0.0 || qTmp[0] > MAXPOS) {
		logTextReadySig("Position out of range", true);
		return false;
	}
	try {
		bool getStatusMem=false;
		_port->clean();
		std::cout<<"MAXIMAL CURRENT = "<<_cube->getMaxCur()<<std::endl;
		while(!_cube->resetCmd()) { rw::common::TimerUtil::sleepMs(100); _port->clean();}
		_port->clean();
		while(!_cube->setTargetVel(VEL)){ rw::common::TimerUtil::sleepMs(100); _port->clean();};
		_port->clean();
		while(!_cube->setTargetAcc(ACC)){ rw::common::TimerUtil::sleepMs(100); _port->clean();};
		_port->clean();
		while(!_cube->moveRampCmd((float)qTmp[0])){ rw::common::TimerUtil::sleepMs(100); _port->clean();};
		unsigned int statusMem=0;
		getStatusMem=false;
		do{
			rw::common::TimerUtil::sleepMs(10);
			getStatusMem = status(statusMem);
			if(statusMem & STATE_POW_INTEGRALERR) {
				logTextReadySig("Motor continuous load limit exceeded", true);
				break;
			}
			if((statusMem & (STATE_BEYOND_SOFT | STATE_BEYOND_HARD))>0) {
				logTextReadySig("Position limits exceeded", true);
				break;
			}
		}while((statusMem & (STATE_MOTION | STATE_RAMP_ACC | STATE_RAMP_STEADY | STATE_RAMP_DEC) )!=0 || (!getStatusMem));
		logTextReadySig("Movements done", true);

	} catch(rw::common::Exception& e) {
		std::cout << "Parallel gripper error: " << e.what() << std::endl;
		return false;
	}
	return true;

}

bool PG70Controller::setGraspPowerPct(const double pct) {
	if(_connected) {
		if(pct < 0.0 || pct > 100.0) {
			logTextReadySig("Invalid grasp power parameter", true);
		} else {
			_graspCurrent = -((float)std::fabs(pct)) * MAXCUR / 100.0f;
			try {
				//_cube->setMaxCur(-_graspCurrent);
				_cube->setCur(_graspCurrent);
			} catch(rw::common::Exception& e) {
				std::cout<< "Parallel gripper error: " << e.what() << std::endl;
				return false;
			}
		}
		return true;
	}
	return false;
}

// Gripper connection status getter
bool PG70Controller::isConnected() {
	if (_cube == NULL || _cubePort == NULL)
		return false;
	//  return _connected;
	return rwhw::Cube::ping(_cube->getCubeID(),_cubePort);
}

bool PG70Controller::status(unsigned int &status) {
	try {
		status =_cube->getCubeState();
		printf ("status %x \n", status);
	} catch(rw::common::Exception& e) {
		return false;
	}
	return true;
}

void PG70Controller::logTextReadySig(const std::string& text, const bool warning) {
	if(warning)
		std::cout<<"Warning: "<< text << std::endl;
	else
		std::cout<<"Information: "<< text<<std::endl;		
}
