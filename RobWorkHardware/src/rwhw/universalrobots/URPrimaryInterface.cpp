/*
 * UniversalRobots.cpp
 *
 *  Created on: Apr 15, 2010
 *      Author: lpe
 */

#include "URPrimaryInterface.hpp"
#include "URCommon.hpp"


#include <rw/common/TimerUtil.hpp>

#include <boost/thread.hpp>

#include <iostream>
#include <string>
#include <float.h>




using namespace rw::models;
using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;

using namespace rwhw;
using namespace boost::asio::ip;



URPrimaryInterface::URPrimaryInterface():
	_haveReceivedSize(false),
	_socket(0),
	_connected(false)
{
}

URPrimaryInterface::~URPrimaryInterface() {
	disconnect();
}

UniversalRobotsData URPrimaryInterface::getLastData() const {
	return _data;
}


void URPrimaryInterface::start() {
	_thread = ownedPtr(new boost::thread(&URPrimaryInterface::run, this));
	_stop = false;
}


void URPrimaryInterface::stop() {
	_stop = true;
}



bool URPrimaryInterface::isConnected() const {
	return _connected;
}


bool URPrimaryInterface::connect(const std::string& ip, unsigned int port) {
	if (_connected) {
		RW_THROW("Already connected. Disconnect before connecting again!");

	}

	try {
		boost::asio::ip::tcp::resolver resolver(_ioService);
		boost::asio::ip::tcp::resolver::query query(ip.c_str(), "");
		boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
		boost::asio::ip::tcp::endpoint ep = (*iter).endpoint();
		ep.port(port);
		//Connecting to server
		_socket = new boost::asio::ip::tcp::socket(_ioService);
		_socket->connect(ep);
	} catch(boost::system::system_error& e) {
		RW_THROW("Unable to connect to command port with message: "<<e.what());
	}

	if (_socket == NULL) {
		_connected = false;
		return false;
	}
	_connected = true;
    return true;
}



bool URPrimaryInterface::sendScript(const std::string& filename)
{
	std::cout<<"Ready to load"<<std::endl;
	std::ifstream infile(filename.c_str());
	std::cout<<"Script Loaded"<<std::endl;
	// get length of file:
	infile.seekg (0, std::ios::end);
	long length = infile.tellg();
	infile.seekg (0, std::ios::beg);

	// allocate memory:
	char *buffer = new char [length];

	// read data as a block:
	infile.read (buffer,length);
	std::cout<<"Send Script "<<buffer<<std::endl;



	std::ostringstream script;
	script << std::setprecision (_commandNumberPrecision);
	script << "def digio():\n";
	script << "  socket_open(\"192.168.100.3\", 33333)"<<std::endl;
	script << "\tsocket_send_string(\"STRING\")\n";
	script << "end\n";
	script << "run program\n";

	std::string out(script.str());
	//return sendCommand(_socketPrimary, out);

	return sendCommand(std::string(buffer));
}



void URPrimaryInterface::disconnect() {
	if(_socket != NULL) {
		_socket->shutdown(boost::asio::socket_base::shutdown_both);
		_socket->close();
		delete _socket;
	}
	_socket = NULL;
}


void URPrimaryInterface::run() {

	while (!_stop) {
		if (_connected) {
			if (readPrimaryInterfacePacket()) {
				boost::this_thread::sleep(boost::posix_time::milliseconds(50));
			}
		} else
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}
		_thread->yield();
	}
}


//Send the script to the robot
bool URPrimaryInterface::sendCommand(const std::string &str) {
	if(!_connected) {
		RW_THROW("Unable to send command before connecting.");
		return false;
	}
	std::cout<<"Send Command\n"<<str<<std::endl;

	_socket->send(boost::asio::buffer(str.c_str(), str.size()));
	return true;
}

//Read incomming data
bool URPrimaryInterface::readPrimaryInterfacePacket() {
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
    uint32_t bytesReady = _socket->available();
    //std::cout<<"bytes ready "<<bytesReady<<std::endl;
	//Check if the data can contain an valid messages length
	if(bytesReady < (uint32_t)sizeof(uint32_t))
		 return false; //Wait for a whole int are ready

	//Get the message length
	if(!_haveReceivedSize) {
		messageLength = URCommon::getUInt32(_socket, messageOffset);
		_haveReceivedSize = true;
	}

	//Check if the data contain the who messages
	//std::cout<<"Compares "<<bytesReady<<" < "<<messageLength<<std::endl;
	if (bytesReady < messageLength)
		return false; //Wait for a who packet are ready

	//Receive the meassage type
	unsigned char messageType = URCommon::getUChar(_socket, messageOffset);

	//Do until the who messages are analysed
	while(messageOffset < messageLength) {
		switch(messageType)
		{
			//Analyse the robot state
			case ROBOT_STATE:
					readRobotsState(messageOffset, messageLength);
				break;

			//Flush the other messages types, as they not yet have any interest for this protocol
			case ROBOT_MESSAGE:
			case HMC_MESSAGE:
			default:
				URCommon::getUChar(_socket, messageOffset);

			break;
		}
	}
	_haveReceivedSize=false;
	return true;
}

//Read the robot start
void URPrimaryInterface::readRobotsState(uint32_t& messageOffset, uint32_t& messageLength) {
	//Temp variabels
	rw::math::Q jointPosition(6);
	rw::math::Q targetJointPosition(6);
	rw::math::Q jointSpeed(6);
	uint16_t tmp16;
	//bool _lastTimeRunningProgram= _data.programRunning;

	//Do until the who messages are analysed
	while(messageOffset<messageLength) 	{
		//Get the packet length
		uint16_t packetLength=URCommon::getUInt32(_socket, messageOffset);
		//Get the packet type
		unsigned char packetType=URCommon::getUChar(_socket, messageOffset);
		switch(packetType) {
		case ROBOT_MODE_DATA:
			//long TimeStamp
			_data.timestamp = URCommon::getUInt64(_socket, messageOffset);
			//std::cout<<"Time Stamp = "<<_data.timestamp<<std::endl;
			//bool physicalRobotsConnected
			_data.physical = URCommon::getBoolean(_socket, messageOffset);

			// bool realRobotsEnabled
			_data.real = URCommon::getBoolean(_socket, messageOffset);

			// bool robot_power_on
			_data.robotPowerOn = URCommon::getBoolean(_socket, messageOffset);
			//std::cout<<"Power On = "<<_data.robotPowerOn<<std::endl;

			// bool emergency_stopped
			_data.emergencyStopped = URCommon::getBoolean(_socket, messageOffset);
			//std::cout<<"Emergency Stopped = "<<_data.emergencyStopped<<std::endl;
			// bool security_stopped
			_data.securityStopped = URCommon::getBoolean(_socket, messageOffset);
			//std::cout<<"Security Stopped = "<<_data.securityStopped<<std::endl;

			// bool program_running
			_data.programRunning = URCommon::getBoolean(_socket, messageOffset);

			// bool program_paused
			_data.programPaused = URCommon::getBoolean(_socket, messageOffset);

			// unsigned char robotMode
//			unsigned char mode = getUchar(messageOffset);
//			if (mode < 0) {
//				mode += 256;
//			}
			_data.robotMode = URCommon::getUChar(_socket, messageOffset);

			//double speedFraction
			_data.speedFraction = URCommon::getDouble(_socket, messageOffset);

			break;

		case JOINT_DATA:
			//For all 6 joints
			for (unsigned int j = 0; j < 6; j++) {
				//get rw::math::Q
				jointPosition[j] = URCommon::getDouble(_socket, messageOffset);
				targetJointPosition[j] = URCommon::getDouble(_socket, messageOffset);
				jointSpeed[j] = URCommon::getDouble(_socket, messageOffset);

				//store Array
				_data.jointCurrent[j] = URCommon::getFloat(_socket, messageOffset);
				_data.jointVoltage[j] = URCommon::getFloat(_socket, messageOffset);
				_data.jointMotorTemperature[j] = URCommon::getFloat(_socket, messageOffset);
				_data.jointMicroTemperature[j] = URCommon::getFloat(_socket, messageOffset);
				_data.jointMode[j] = URCommon::getUChar(_socket, messageOffset);
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
			_data.analogInputRange[2] = URCommon::getUChar(_socket, messageOffset);
			_data.analogInputRange[3] = URCommon::getUChar(_socket, messageOffset);

			// double Analog input
			_data.analogIn[2] = URCommon::getDouble(_socket, messageOffset);
			_data.analogIn[3] = URCommon::getDouble(_socket, messageOffset);

			// float toolVoltage48V
			_data.toolVoltage48V = URCommon::getFloat(_socket, messageOffset);

			// unsigned char tool_output_voltage (can only be 0, 12 or 24)
			_data.toolOutputVoltage = URCommon::getUChar(_socket, messageOffset);

			// float tool_current
			_data.toolCurrent = URCommon::getFloat(_socket, messageOffset);

			// float toolTemperature;
			_data.toolTemperature = URCommon::getFloat(_socket, messageOffset);

			// uchar tool mode
			_data.toolMode = URCommon::getUChar(_socket, messageOffset);
			break;

		case MASTERBOARD_DATA:
			// unsigned short digitalInputBits;
			tmp16 = URCommon::getUInt16(_socket, messageOffset);
			for(unsigned int i=0; i<10; ++i) // Tool bits are actually included here
				_data.digitalIn[i] = extractBoolean(tmp16, i);

			// unsigned short digitalOutputBits;
			tmp16 = URCommon::getUInt16(_socket, messageOffset);
			for(unsigned int i=0; i<10; ++i) // Tool bits are actually included here
				_data.digitalOut[i] = extractBoolean(tmp16, i);

			// unsigned char analogInputRange[4]
			_data.analogInputRange[0] = URCommon::getUChar(_socket, messageOffset);
			_data.analogInputRange[1] = URCommon::getUChar(_socket, messageOffset);

			//double analogInput[4];
			_data.analogIn[0] = URCommon::getDouble(_socket, messageOffset);
			_data.analogIn[1] = URCommon::getDouble(_socket, messageOffset);

			// char analogOutputDomain[2]
			_data.analogOutputDomain[0] = URCommon::getUChar(_socket, messageOffset);
			_data.analogOutputDomain[1] = URCommon::getUChar(_socket, messageOffset);

			//double analogOutput[2];
			_data.analogOut[0] = URCommon::getDouble(_socket, messageOffset);
			_data.analogOut[1] = URCommon::getDouble(_socket, messageOffset);

			// float masterTemperature;
			_data.masterTemperature = URCommon::getFloat(_socket, messageOffset);

			// float robotVoltage48V;
			_data.robotVoltage48V = URCommon::getFloat(_socket, messageOffset);

			// float robotCurrent;
			_data.robotCurrent = URCommon::getFloat(_socket, messageOffset);

			// float masterIOCurrent;
			_data.masterIOCurrent = URCommon::getFloat(_socket, messageOffset);
			break;

		case CARTESIAN_INFO:
			_data.toolPosition = URCommon::getVector3D(_socket, messageOffset);
			_data.toolAxixAngle = URCommon::getVector3D(_socket, messageOffset);
			break;

		case LASER_POINTER_POSITION:
			_data.laserPointerPosition = URCommon::getVector3D(_socket, messageOffset);
			break;
		default:
			for(unsigned int i = 5; i<packetLength; i++)
				URCommon::getUChar(_socket, messageOffset);
			break;
		}
	}
}


//Extract many booleans, max 16
bool URPrimaryInterface::extractBoolean(uint16_t input, unsigned int bitNumber) {
	uint16_t filter = 1<<bitNumber;
	bool output = (input & filter)>0;
	return output;
}

