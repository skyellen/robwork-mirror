/*
 * UniversalRobot.cpp
 *
 *  Created on: Apr 15, 2010
 *      Author: lpe
 */

#include "UniversalRobot.hpp"



#include <rw/common/TimerUtil.hpp>

#include <iostream>
#include <string>
#include <float.h>




using namespace rw::models;
using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;



UniversalRobot::UniversalRobot():
    _socket(0),
	_connected(false),
	_haveReceivedSize(false)
{
}

UniversalRobot::~UniversalRobot() {
	if (_connected)
		disconnect();
}

bool UniversalRobot::isConnected() const {
	return _connected;
}

bool UniversalRobot::connect(const std::string& host, int port) {
	if(!connectSocket(host, port)) {
//		rw::common::log(Log::Error)<<"Can not make a socket connection to " << _device->getName() << endlog();
		return false;
	}
    return true;
}

void UniversalRobot::disconnect() {
	disconnectSocket();
}


void UniversalRobot::update() {
	while(readPacket());
}


bool UniversalRobot::moveTo(const rw::math::Q& q) {
	QPath path;
	path.push_back(q);
	return moveJ(path);
}
	
	


bool UniversalRobot::executePath(const rw::trajectory::QPath& path) {
	bool success = true;
	BOOST_FOREACH(const Q& q, path) {
		if (!moveTo(q))
			return false;
	}
	return true;
}
    


bool UniversalRobot::executeTrajectory(rwlibs::task::QTask::Ptr task, double speed, int id) {
//	log(Info)<<"Receive task"<<endlog();

	//Get device info from task
	//int taskId = task->getPropertyMap().get<int>("Id", -1);
	//std::string device = task->getPropertyMap().get<std::string>("Device", "");

	//Get motions
	rw::trajectory::QPath path;
	std::vector<double> blends;
	std::vector<rwlibs::task::QMotion::Ptr> motions = task->getMotions();

	//The first configuration in the path is the actual configuration
	// TODO: Dangerous: replace with motions[0]->start()
	path.push_back(_data.jointPosition);
	//Load the movements
	for(unsigned int i=0; i<motions.size(); i++) {
		path.push_back(motions.at(i)->end());
	}
	//Blend size = path.size()-2
	for(unsigned int i=0; i<motions.size()-1; i++) {
		blends.push_back(motions.at(i)->getPropertyMap().get<double>("Blend", 0.1));
	}

    try {
		double v = speed;
		if (v > 1) {
			v = 1;
		}
		//Send command
		_trajectoryId = id;
		servoJ(path, blends, v,v);

    } catch (const rw::common::Exception& exp) {
		return false;
	}
	return true;
}



//Connect to the socket
bool UniversalRobot::connectSocket(const std::string &ip, unsigned int port) {
	_connected = false;
	try {		
		boost::asio::ip::tcp::resolver resolver(_io_service);
		boost::asio::ip::tcp::resolver::query query(ip.c_str(), "");
		boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
		boost::asio::ip::tcp::endpoint ep = (*iter).endpoint();
		ep.port(port);
		//Connecting to PR server
		_socket = new boost::asio::ip::tcp::socket(_io_service);
		_socket->connect(ep);
		//Connected
//		log(Info)<< _device->getName() << " connected to " << ip <<":"<<port<<endlog();
		_connected = true;
	} catch(boost::system::system_error& e) {
		RW_THROW("Unable to connect with message: "<<e.what());
//		log(Error) << "Connection error to " << ip << ":" << port << " : " << e.what() << endlog();
	}
	return _connected;
}

//Disconnect the socket
void UniversalRobot::disconnectSocket() {
	if(_socket != NULL) {
		_socket->shutdown(boost::asio::socket_base::shutdown_both);
		_socket->close();
		delete _socket;
//		log(Info)<< _device->getName() << " disconnected" <<endlog();
	}
	_socket = NULL;
	_connected = false;
}

bool UniversalRobot::getChar(char* output) {
	return _socket->read_some(boost::asio::buffer(output, 1));
}

//Send the script to the robot
bool UniversalRobot::sendCommand(std::string &str) {
	if(!_connected) {
//		log(Error)<< "Can not send command to " << _device->getName() << ", socket not connected" <<endlog();
		return false;
	}

//	log(Info) << "script: " << str << endlog();
	_socket->send(boost::asio::buffer(str.c_str(), str.size()));
	return true;
}

void UniversalRobot::pathToScriptString(const rw::trajectory::QPath& path, std::ostringstream &stream) {
	stream << "[";
	for(int i = 0; i < path.size()-1; i++) {
		for(int j = 0; j < path[i].size(); j++) {
			stream << path[i][j] << ",";
		}
	}
	for(int j = 0; j < path[path.size()-1].size()-1; j++) {
		stream << path[path.size()-1][j] << ",";
	}
	stream << path[path.size()-1][path[path.size()-1].size()-1] << "]";
}

// Parse the rw::math::Q as a string suitable for scripts
void UniversalRobot::qToScriptString(const rw::math::Q& q, int index, std::ostringstream &stream) {
	stream << "q" << index << "=[";
	for(unsigned int i = 0; i < q.size() - 1; i++) {
		stream << q[i] << ", ";
	}
	stream << q[q.size() - 1] << "]";
}

// Move linearly in joint space along a path
bool UniversalRobot::servoJ(const rw::trajectory::QPath& path, const std::vector<double>& betas, double velScale, double accScale) {
	if(betas.size() != path.size()-2)
		return false;
	if(path.size() == 2)
		return moveJ(path);

	//Convert from angles to "encoder angles"
	//rw::trajectory::QPath pathOut = path;//rw::models::EncoderDecentralization::calcEncoderAngle(path, _tau, _sigma);

	//Generate script
	std::ostringstream script;
	script << std::setprecision (_commandNumberPrecision);
	script << "def movePath():\n";
	script << "\tpath=";
	pathToScriptString(path,script);
	script << "\n";
	script << "\tbeta=[";
	for(unsigned int i = 0; i < betas.size()-1; ++i){
		script << betas[i] << ",";
	}
	script << betas[betas.size()-1] << "]\n";
	script << "\tvScale=" << velScale << "\n";
	script << "\taScale=" << accScale << "\n";
	script << "\tservoj(path,beta,vScale,aScale)\n";
	script << "end\n";
	script << "run program\n";

	std::string out(script.str());
	return sendCommand(out);
}

/*
bool UniversalRobot::moveJ(const rw::math::Transform3D<>& transform) {
	//Generate script
	std::ostringstream script;
	script << std::setprecision (_commandNumberPrecision);

	//Start script
	script << "def movePose():\n";
	//Move by the configurations
	
	EAA<> eaa(transform.R());
	script << "\tmovel(p["<<transform.P()(0)<<","<<transform.P()(1)<<","<<transform.P()(2)<<","<<eaa(0)<<","<<eaa(1)<<","<<eaa(2)<<", 0.2, 0.1, 0, 0])\n";
	

	//Stop script
	script << "end\n";
	//Run the script
	script << "run program\n";


	//Send to thread
	std::string out(script.str());
	return sendCommand(out);

}*/

// Move linearly in joint space along a path
bool UniversalRobot::moveJ(const rw::trajectory::QPath& path) {
	if(path.empty())
		return false;

	//Convert from angles to "encoder angles"
	//rw::trajectory::QPath pathOut = rw::models::EncoderDecentralization::calcEncoderAngle(path, _tau, _sigma);

	//Generate script
	std::ostringstream script;
	script << std::setprecision (_commandNumberPrecision);

	//Start script
	script << "def movePath():\n";
	//Configuration for the path
	for(unsigned int i = 0; i < path.size(); i++) {
		script << "\t";
		qToScriptString(path[i],i, script);
		script << "\n";
	}
	//Move by the configurations
	for(unsigned int i = 0; i < path.size(); i++) {
		script << "\tmovej(q" << i << ")\n";
	}

	//Stop script
	script << "end\n";
	//Run the script
	script << "run program\n";


	//Send to thread
	std::string out(script.str());
	return sendCommand(out);
}

//Read incomming data
bool UniversalRobot::readPacket() {
	if(!_connected) {
		return false;
	}

    /* Messages are of the form:
     *  - Length of entire message (int)
     *  - type of message (byte)
     *    - length of package 1 (1 int)
     *    - type of package 1 (1 byte)
     *    - data for package 1 (many bytes)
     *   - repeat for package 2
     *  - end of message
     **/

	//Initialise variables
	if(!_haveReceivedSize) {
		messageOffset = 0;
		messageLength = 0;
	}

    //Get the length of the available data
    uint32 bytesReady = _socket->available();
	//Check if the data can contain an valid messages length
	if(bytesReady < (uint32)sizeof(uint32))
		 return false; //Wait for a who int are ready

	//Get the message length
	if(!_haveReceivedSize) {
		messageLength = getUINT32(messageOffset);
		_haveReceivedSize = true;
	}

	//Check if the data contain the who messages
	if (bytesReady < messageLength)
		return false; //Wait for a who packet are ready

	//Receive the meassage type
	unsigned char messageType = getUchar(messageOffset);

	//Do until the who messages are analysed
	while(messageOffset < messageLength) {
		switch(messageType)
		{
			//Analyse the robot state
			case ROBOT_STATE:
					readRobotState(messageOffset, messageLength);
				break;

			//Flush the other messages types, as they not yet have any interest for this protocol
			case ROBOT_MESSAGE:
			case HMC_MESSAGE:
			default:
				getUchar(messageOffset);

			break;
		}
	}
	_haveReceivedSize=false;
	return true;
}

//Read the robot start
void UniversalRobot::readRobotState(uint32& messageOffset, uint32& messageLength) {
	//Temp variabels
	rw::math::Q jointPosition(6);
	rw::math::Q targetJointPosition(6);
	rw::math::Q jointSpeed(6);
	uint16 tmp16;
	bool _lastTimeRunningProgram= _data.programRunning;

	//Do until the who messages are analysed
	while(messageOffset<messageLength) 	{
		//Get the packet length
		uint16 packetLength=getUINT32(messageOffset);
		//Get the packet type
		unsigned char packetType=getUchar(messageOffset);

		switch(packetType) {
		case ROBOT_MODE_DATA:
			//long TimeStamp
			_data.timestamp = getLong(messageOffset);

			//bool physicalRobotConnected
			_data.physical = getBoolean(messageOffset);

			// bool realRobotEnabled
			_data.real = getBoolean(messageOffset);

			// bool robot_power_on
			_data.robotPowerOn = getBoolean(messageOffset);

			// bool emergency_stopped
			_data.emergencyStopped = getBoolean(messageOffset);

			// bool security_stopped
			_data.securityStopped = getBoolean(messageOffset);

			// bool program_running
			_data.programRunning = getBoolean(messageOffset);

			// bool program_paused
			_data.programPaused = getBoolean(messageOffset);

			// unsigned char robotMode
//			unsigned char mode = getUchar(messageOffset);
//			if (mode < 0) {
//				mode += 256;
//			}
			_data.robotMode = getUchar(messageOffset);

			//double speedFraction
			_data.speedFraction = getDouble(messageOffset);

			break;

		case JOINT_DATA:
			//For all 6 joints
			for (unsigned int j = 0; j < 6; j++) {
				//get rw::math::Q
				jointPosition[j] = getDouble(messageOffset);
				targetJointPosition[j] = getDouble(messageOffset);
				jointSpeed[j] = getDouble(messageOffset);

				//store Array
				_data.jointCurrent[j] = getFloat(messageOffset);
				_data.jointVoltage[j] = getFloat(messageOffset);
				_data.jointMotorTemperature[j] = getFloat(messageOffset);
				_data.jointMicroTemperature[j] = getFloat(messageOffset);
				_data.jointMode[j] = getUchar(messageOffset);
			}
			//Store rw::math::Q
			//Encoder adjustment
			//jointPosition = rw::models::EncoderDecentralization::calcRealAngle(jointPosition, _tau, _sigma);
			//targetJointPosition = rw::models::EncoderDecentralization::calcRealAngle(targetJointPosition, _tau, _sigma);

			_data.jointPosition = jointPosition;
			_data.targetJointPosition = targetJointPosition;
			_data.jointSpeed = jointSpeed;
			break;

		case TOOL_DATA:
			//unsigned char Analog input range
			_data.analogInputRange[2] = getUchar(messageOffset);
			_data.analogInputRange[3] = getUchar(messageOffset);

			// double Analog input
			_data.analogIn[2] = getDouble(messageOffset);
			_data.analogIn[3] = getDouble(messageOffset);

			// float toolVoltage48V
			_data.toolVoltage48V = getFloat(messageOffset);

			// unsigned char tool_output_voltage (can only be 0, 12 or 24)
			_data.toolOutputVoltage = getUchar(messageOffset);

			// float tool_current
			_data.toolCurrent = getFloat(messageOffset);

			// float toolTemperature;
			_data.toolTemperature = getFloat(messageOffset);

			// uchar tool mode
			_data.toolMode = getUchar(messageOffset);
			break;

		case MASTERBOARD_DATA:
			// unsigned short digitalInputBits;
			tmp16 = getUINT16(messageOffset);
			for(unsigned int i=0; i<10; ++i) // Tool bits are actually included here
				_data.digitalIn[i] = extractBoolean(tmp16, i);

			// unsigned short digitalOutputBits;
			tmp16 = getUINT16(messageOffset);
			for(unsigned int i=0; i<10; ++i) // Tool bits are actually included here
				_data.digitalOut[i] = extractBoolean(tmp16, i);

			// unsigned char analogInputRange[4]
			_data.analogInputRange[0] = getUchar(messageOffset);
			_data.analogInputRange[1] = getUchar(messageOffset);

			//double analogInput[4];
			_data.analogIn[0] = getDouble(messageOffset);
			_data.analogIn[1] = getDouble(messageOffset);

			// char analogOutputDomain[2]
			_data.analogOutputDomain[0] = getUchar(messageOffset);
			_data.analogOutputDomain[1] = getUchar(messageOffset);

			//double analogOutput[2];
			_data.analogOut[0] = getDouble(messageOffset);
			_data.analogOut[1] = getDouble(messageOffset);

			// float masterTemperature;
			_data.masterTemperature = getFloat(messageOffset);

			// float robotVoltage48V;
			_data.robotVoltage48V = getFloat(messageOffset);

			// float robotCurrent;
			_data.robotCurrent = getFloat(messageOffset);

			// float masterIOCurrent;
			_data.masterIOCurrent = getFloat(messageOffset);
			break;

		case CARTESIAN_INFO:
			_data.toolPosition = getVector3D(messageOffset);
			_data.toolAxixAngle = getVector3D(messageOffset);
			break;

		case LASER_POINTER_POSITION:
			_data.laserPointerPosition = getVector3D(messageOffset);
			break;
		default:
			for(unsigned int i = 5; i<packetLength; i++)
				getUchar(messageOffset);
			break;
		}
	}

}



//Extract a unsigned char
unsigned char UniversalRobot::getUchar(uint32 &messageOffset) {
	unsigned char output = 0;
	getChar((char*)&output+0);
	messageOffset += 1;
	return output;
}

//Extract a 16 bit unsigned int
uint16 UniversalRobot::getUINT16(uint32 &messageOffset) {
	uint16 output = 0;
	getChar((char*)&output+1);
	getChar((char*)&output+0);
	messageOffset += 2;
	return output;
}

//Extract a 32 bit unsigned int
uint32 UniversalRobot::getUINT32(uint32 &messageOffset) {
	uint32 output = 0;
	getChar((char*)&output+3);
	getChar((char*)&output+2);
	getChar((char*)&output+1);
	getChar((char*)&output+0);
	messageOffset += 4;
	return output;
}

//Extract a float
float UniversalRobot::getFloat(uint32 &messageOffset) {
	float output = 0;
	getChar((char*)&output+3);
	getChar((char*)&output+2);
	getChar((char*)&output+1);
	getChar((char*)&output+0);
	messageOffset += 4;
	return output;
}

//Extract a double
double UniversalRobot::getDouble(uint32 &messageOffset) {
	double output = 0;
	getChar((char*)&output+7);
	getChar((char*)&output+6);
	getChar((char*)&output+5);
	getChar((char*)&output+4);
	getChar((char*)&output+3);
	getChar((char*)&output+2);
	getChar((char*)&output+1);
	getChar((char*)&output+0);
	messageOffset += 8;
	return output;
}

//Extract a Long
long UniversalRobot::getLong(uint32 &messageOffset) {
	long output = 0;
	getChar((char*)&output+7);
	getChar((char*)&output+6);
	getChar((char*)&output+5);
	getChar((char*)&output+4);
	getChar((char*)&output+3);
	getChar((char*)&output+2);
	getChar((char*)&output+1);
	getChar((char*)&output+0);
	messageOffset += 8;
	return output;
}

//Extract a one boolean
bool UniversalRobot::getBoolean(uint32 &messageOffset) {
	char tmp = 0;
	getChar((char*)&tmp);
	bool output = (tmp & 1)>0;
	messageOffset += 1;
	return output;
}

//Extract many booleans, max 16
bool UniversalRobot::extractBoolean(uint16 input, unsigned int bitNumber) {
	uint16 filter = 1<<bitNumber;
	bool output = (input & filter)>0;
	return output;
}

//Extract a 3d vector
rw::math::Vector3D<double> UniversalRobot::getVector3D(uint32 &messageOffset) {
	rw::math::Vector3D<double> output;
	output[0] = getDouble(messageOffset);
	output[1] = getDouble(messageOffset);
	output[2] = getDouble(messageOffset);
	return output;
}
