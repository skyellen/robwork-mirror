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

#include "URPrimaryInterface.hpp"
#include "URCommon.hpp"

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Log.hpp>

#include <boost/thread.hpp>
#include <boost/asio/write.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <float.h>

using namespace rw::math;
using namespace rw::common;
using namespace rwhw;
using namespace boost::asio::ip;

URPrimaryInterface::URPrimaryInterface() :
	_thread(NULL),
	_stop(false),
	_haveReceivedSize(false),
	_messageLength(0),
	_messageOffset(0),
	_socket(0),
	_hostName(""),
	_connected(false),
	_hasURData(false)
{
}

URPrimaryInterface::~URPrimaryInterface() {
	disconnect();
	stop();
}

double URPrimaryInterface::driverTime() const {
	return URCommon::driverTimeStamp();
}

bool URPrimaryInterface::hasData() const {
	return _hasURData;
}


UniversalRobotsData URPrimaryInterface::getLastData() const {
	boost::mutex::scoped_lock lock(_mutex);
	return _data;
}


void URPrimaryInterface::start() {
	_thread = ownedPtr(new boost::thread(&URPrimaryInterface::run, this));
	_stop = false;
}


void URPrimaryInterface::stop() {
	_stop = true;
	if (_thread != NULL) {
		// Give the thread one second to stop
		if (!_thread->timed_join(boost::posix_time::seconds(1))) {
			// Failure, interrupt
			RW_WARN("Interrupting URPrimaryInterface thread...");
			_thread->interrupt();
			if (!_thread->timed_join(boost::posix_time::seconds(1)))
				RW_THROW("Failed to interrupt URPrimaryInterface thread");
		}
		_thread = NULL;
	}
}



bool URPrimaryInterface::isConnected() const {
	return _connected && !_lostConnection;
}


void URPrimaryInterface::connect(const std::string& ip, unsigned int port) {
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
		_connected = false;
		RW_THROW("Unable to connect to command port with message: "<<e.what());
	}

	_connected = true;	
}


std::string URPrimaryInterface::getLocalIP() {
	if (_socket->is_open()) {
		boost::asio::ip::address addr = _socket->local_endpoint().address();	
		return addr.to_string();
	}
	RW_THROW("Unable to obtain IP before the connection to the UR has been established");
}

bool URPrimaryInterface::sendScriptFile(const std::string& filename)
{
	std::ifstream infile(filename.c_str());
	// get length of file:
	infile.seekg (0, std::ios::end);
	std::streampos length = infile.tellg();
	infile.seekg (0, std::ios::beg);

	// allocate memory:
	char *buffer = new char [(int)length+1];

	// read data as a block:
	infile.read (buffer,length);
    buffer[(int)length] = 0;
	
    const bool ret = sendScript(buffer);
    delete[] buffer;
    return ret;

}

bool URPrimaryInterface::sendScript(const std::string& script) 
{
	return sendCommand(script);
}



void URPrimaryInterface::disconnect() {
	if(_socket != NULL) {
		_socket->shutdown(boost::asio::socket_base::shutdown_both);
		_socket->close();
		delete _socket;
	}
	_connected = false;
	_socket = NULL;
}


void URPrimaryInterface::run() {

	while (!_stop) {
		if (_connected) {
			if (readPrimaryInterfacePacket()) {
				boost::this_thread::sleep(boost::posix_time::milliseconds(20));
			}


			long time = static_cast<long>(TimerUtil::currentTimeMs());
			if(time-_lastPackageTime>1000){
				_lostConnection = true;
			} else {
				_lostConnection = false;
			}


		} else
		{
			_lastPackageTime = 1234;
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}
		_thread->yield();
	}
	RW_LOG_DEBUG("URPrimaryInterface thread stopped ");
	_lastPackageTime = 666;
}


//Send the script to the robot
bool URPrimaryInterface::sendCommand(const std::string &str) {
	if(!_connected) {
		RW_THROW("Unable to send command before connecting.");
		return false;
	}

    std::size_t bytesTransfered = 0;
    bytesTransfered = boost::asio::write(*_socket, boost::asio::buffer(str.c_str(), str.size()));
    RW_LOG_DEBUG("Bytes to be transfered '" << str.size() << "' and bytes actually transfered '" << bytesTransfered << "'");

    if (str.size() == bytesTransfered) {
        return true;
    } else {
        return false;
    }
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
		_messageOffset = 0;
		_messageLength = 0;
	}

    //Get the length of the available data
    uint32_t bytesReady = static_cast<uint32_t>(_socket->available());
	//Check if the data can contain an valid messages length
	if(bytesReady < (uint32_t)sizeof(uint32_t))
		 return false; //Wait for a whole int are ready

	//Get the message length
	if(!_haveReceivedSize) {
		_messageLength = URCommon::getUInt32(_socket, _messageOffset);
		_haveReceivedSize = true;
	}
	_lastPackageTime = static_cast<long>(TimerUtil::currentTimeMs());
	
	if (bytesReady < _messageLength)
		return false; //Wait for a who packet are ready

	//Receive the meassage type
	unsigned char messageType = URCommon::getUChar(_socket, _messageOffset);

	// read the rest of the message into a buffer
	unsigned int dataLength = _messageLength - _messageOffset;
	if(_dataStorage.size()<dataLength)
		_dataStorage.resize(dataLength);

	URCommon::getData(_socket,  _messageLength-_messageOffset,_dataStorage);
	_messageOffset = 0;
	while(_messageOffset<dataLength)
	{
		switch(messageType)
		{
			//Analyse the robot state
			case ROBOT_STATE:
				readRobotsState( _dataStorage);
				_hasURData = true;
				break;
			//Flush the other messages types, as they not yet have any interest for this protocol
			case ROBOT_MESSAGE:
				readRobotMessage(_dataStorage, dataLength);
				break;
			case HMC_MESSAGE:
			default:
				uint32_t msglength =  URCommon::getUInt32(_dataStorage, _messageOffset);
				_messageOffset += msglength;
			break;
		}
	}
	
	_haveReceivedSize=false;
	return true;
}

void URPrimaryInterface::readRobotMessage(const std::vector<char>& data, unsigned int messageLength)
{
	uint32_t startOffset = _messageOffset;
	_data.controllerTimeStamp = static_cast<long>(URCommon::getUInt64(data, _messageOffset));
	URCommon::getUChar(data, _messageOffset);
	char robotMessageType = URCommon::getUChar(data, _messageOffset);

	switch (robotMessageType) {

	case URMessage::ROBOT_MESSAGE_VERSION: {
		int projectNameSize = (int)URCommon::getUChar(data, _messageOffset);
		std::string projectName = URCommon::getString(data, projectNameSize, _messageOffset);
		int majorVersion = (int)URCommon::getUChar(data, _messageOffset);
		int minorVersion = (int)URCommon::getUChar(data, _messageOffset);
		_messages.push(URMessage(URMessage::ROBOT_MESSAGE_VERSION, majorVersion, minorVersion, "Project Name: ", projectName));
		_messageOffset = startOffset + messageLength;
		}
		break;
	case URMessage::ROBOT_MESSAGE_SECURITY: {
		int robotMessageCode = URCommon::getUInt32(data, _messageOffset);
		int robotMessageArgument = URCommon::getUInt32(data, _messageOffset);
		std::string textmsg;
		char safetyModeType = URCommon::getUChar(data, _messageOffset);
		std::string safetyModeTypeText = "";
		switch (safetyModeType) {
		case 1: 
			safetyModeTypeText = "NORMAL";
			break;
		case 2:
			safetyModeTypeText = "REDUCED";
			break;
		case 3:
			safetyModeTypeText = "PROTECTIVE STOP";
			break;
		case 4:
			safetyModeTypeText = "RECOVERY";
			break;
		case 5:
			safetyModeTypeText = "SAFE GUARD STOP";
			break;
		case 6:
			safetyModeTypeText = "SYSTEM EMERGENCY STOP";
			break;
		case 7:
			safetyModeTypeText = "ROBOT EMERGENCY STOP";
			break;
		case 8:
			safetyModeTypeText = "VIOLATION";
			break;
		case 9:
			safetyModeTypeText = "FAULT";
			break;
		default:
			safetyModeTypeText = "UNKNOWN";
			break;
		}
		if (messageLength > _messageOffset) {
			size_t txtLength = messageLength - _messageOffset;
			char* msg = new char[txtLength + 1];
			msg[txtLength] = 0;

			for (size_t i = 0; i<txtLength; i++) {
				msg[i] = URCommon::getUChar(data, _messageOffset);
			}
			textmsg = msg;
			delete[] msg;
		}
		_messages.push(URMessage(URMessage::ROBOT_MESSAGE_SECURITY, robotMessageCode, robotMessageArgument, textmsg, safetyModeTypeText));
	}
		break;
	case URMessage::ROBOT_MESSAGE_ERROR_CODE: {
		int robotMessageCode = URCommon::getUInt32(data, _messageOffset);
		int robotMessageArgument = URCommon::getUInt32(data, _messageOffset);
		int warningLevel = URCommon::getUInt32(data, _messageOffset);
		std::string levelMsg = "";
		switch (warningLevel) {
		case 1:
			levelMsg = "INFO";
			break;
		case 2: 
			levelMsg = "WARNING";
			break;
		case 3:
			levelMsg = "VIOLATION";
			break;
		case 4:
			levelMsg = "FAULT";
			break;
		default:
			levelMsg = "UNKNOWN";
			break;
		}
		std::string textmsg;
		if (messageLength > _messageOffset) {
			size_t txtLength = messageLength - _messageOffset;
			char* msg = new char[txtLength + 1];
			msg[txtLength] = 0;

			for (size_t i = 0; i<txtLength; i++) {
				msg[i] = URCommon::getUChar(data, _messageOffset);
			}
			textmsg = msg;
			delete[] msg;
		}
		_messages.push(URMessage(URMessage::ROBOT_MESSAGE_ERROR_CODE, robotMessageCode, robotMessageArgument, levelMsg, textmsg));
	}
	break;
	case URMessage::ROBOT_MESSAGE_KEY:
		_messages.push(URMessage(URMessage::ROBOT_MESSAGE_KEY, 0, 0, "ROBOT MESSAGE KEY", ""));
		break;
	case URMessage::ROBOT_MESSAGE_PROGRAM_LABEL:
		_messages.push(URMessage(URMessage::ROBOT_MESSAGE_PROGRAM_LABEL, 0, 0, "ROBOT_MESSAGE_PROGRAM_LABEL", ""));
		break;
	case URMessage::ROBOT_MESSAGE_POPUP: {
		bool warning = URCommon::getBoolean(data, _messageOffset);
		bool error = URCommon::getBoolean(data, _messageOffset);
		unsigned int titleSize = URCommon::getUInt32(data, _messageOffset);
		char* title = new char[titleSize+1];
		title[titleSize] = 0;
		for (unsigned int i = 0; i<titleSize; i++) {
			title[i] = URCommon::getUChar(data, _messageOffset);
		}

		size_t msgLength = messageLength - _messageOffset;
		char* msg = new char[msgLength + 1];
		msg[msgLength] = 0;
		for (size_t i = 0; i<msgLength; i++) {
			msg[i] = URCommon::getUChar(data, _messageOffset);
		}
		_messages.push(URMessage(URMessage::ROBOT_MESSAGE_PROGRAM_LABEL, warning, error, std::string(title), std::string(msg)));
		delete[] msg;
		delete[] title;

		}
		break;
	case URMessage::ROBOT_MESSAGE_TEXT: {
		std::string textmsg;
		if (messageLength > _messageOffset) {
			size_t txtLength = messageLength - _messageOffset;
			char* msg = new char[txtLength + 1];
			msg[txtLength] = 0;

			for (size_t i = 0; i<txtLength; i++) {
				msg[i] = URCommon::getUChar(data, _messageOffset);
			}
			textmsg = msg;
			delete[] msg;
		}
		_messages.push(URMessage(URMessage::ROBOT_MESSAGE_TEXT, 0, 0, "TEXT MESSAGE", textmsg));
		}
		break;
	case URMessage::ROBOT_MESSAGE_VARIABLE:
		break;
	default:
		break;
	}
	_messageOffset = startOffset + messageLength;
}

//Read the robot start
void URPrimaryInterface::readRobotsState(const std::vector<char>& data) {
	boost::mutex::scoped_lock lock(_mutex);

	//Temp variabels
	rw::math::Q jointPosition(6);
	rw::math::Q targetJointPosition(6);
	rw::math::Q jointSpeed(6);
	uint16_t tmp16;

	_data.driverTimeStamp = driverTime();
	//Get the packet length
	uint32_t packetLength =  URCommon::getUInt32(data, _messageOffset);
	//Get the packet type
	unsigned char packetType= URCommon::getUChar(data, _messageOffset);

	switch(packetType) {
	case ROBOT_MODE_DATA:
		_data.controllerTimeStamp = static_cast<long>(URCommon::getUInt64(data, _messageOffset));
		_data.physical = URCommon::getBoolean(data, _messageOffset);
		_data.real = URCommon::getBoolean(data, _messageOffset);
		_data.robotPowerOn = URCommon::getBoolean(data, _messageOffset);
		_data.emergencyStopped = URCommon::getBoolean(data, _messageOffset);
		_data.securityStopped = URCommon::getBoolean(data, _messageOffset);
		_data.programRunning = URCommon::getBoolean(data, _messageOffset);
		_data.programPaused = URCommon::getBoolean(data, _messageOffset);
		_data.robotMode = URCommon::getUChar(data, _messageOffset);
		_data.speedFraction = URCommon::getDouble(data, _messageOffset);
		break;

	case JOINT_DATA:
		//For all 6 joints
		for (unsigned int j = 0; j < 6; j++) {
			//get rw::math::Q
			jointPosition[j] = URCommon::getDouble(data, _messageOffset);
			targetJointPosition[j] = URCommon::getDouble(data, _messageOffset);
			jointSpeed[j] = URCommon::getDouble(data, _messageOffset);

			//store Array
			_data.jointCurrent[j] = static_cast<float>(URCommon::getFloat(data, _messageOffset));
			_data.jointVoltage[j] = static_cast<float>(URCommon::getFloat(data, _messageOffset));
			_data.jointMotorTemperature[j] = static_cast<float>(URCommon::getFloat(data, _messageOffset));
			_data.jointMicroTemperature[j] = static_cast<float>(URCommon::getFloat(data, _messageOffset));
			_data.jointMode[j] = URCommon::getUChar(data, _messageOffset);
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
		_data.analogInputRange[2] = URCommon::getUChar(data, _messageOffset);
		_data.analogInputRange[3] = URCommon::getUChar(data, _messageOffset);

		// double Analog input
		_data.analogIn[2] = URCommon::getDouble(data, _messageOffset);
		_data.analogIn[3] = URCommon::getDouble(data, _messageOffset);

		// float toolVoltage48V
		_data.toolVoltage48V = static_cast<float>(URCommon::getFloat(data, _messageOffset));

		// unsigned char tool_output_voltage (can only be 0, 12 or 24)
		_data.toolOutputVoltage = URCommon::getUChar(data, _messageOffset);

		// float tool_current
		_data.toolCurrent = static_cast<float>(URCommon::getFloat(data, _messageOffset));

		// float toolTemperature;
		_data.toolTemperature = static_cast<float>(URCommon::getFloat(data, _messageOffset));

		// uchar tool mode
		_data.toolMode = URCommon::getUChar(data, _messageOffset);
		break;

	case MASTERBOARD_DATA:
	{
		// unsigned short digitalInputBits;
		tmp16 = URCommon::getUInt16(data, _messageOffset);
		for(unsigned int i=0; i<10; ++i) // Tool bits are actually included here
			_data.digitalIn[i] = extractBoolean(tmp16, i);

		// unsigned short digitalOutputBits;
		tmp16 = URCommon::getUInt16(data, _messageOffset);
		for(unsigned int i=0; i<10; ++i) // Tool bits are actually included here
			_data.digitalOut[i] = extractBoolean(tmp16, i);

		// unsigned char analogInputRange[4]
		_data.analogInputRange[0] = URCommon::getUChar(data, _messageOffset);
		_data.analogInputRange[1] = URCommon::getUChar(data, _messageOffset);

		//double analogInput[4];
		_data.analogIn[0] = URCommon::getDouble(data, _messageOffset);
		_data.analogIn[1] = URCommon::getDouble(data, _messageOffset);

		// char analogOutputDomain[2]
		_data.analogOutputDomain[0] = URCommon::getUChar(data, _messageOffset);
		_data.analogOutputDomain[1] = URCommon::getUChar(data, _messageOffset);

		//double analogOutput[2];
		_data.analogOut[0] = URCommon::getDouble(data, _messageOffset);
		_data.analogOut[1] = URCommon::getDouble(data, _messageOffset);

		// float masterTemperature;
		_data.masterTemperature = static_cast<float>(URCommon::getFloat(data, _messageOffset));

		// float robotVoltage48V;
		_data.robotVoltage48V = static_cast<float>(URCommon::getFloat(data, _messageOffset));

		// float robotCurrent;
		_data.robotCurrent = static_cast<float>(URCommon::getFloat(data, _messageOffset));

		// float masterIOCurrent;
		_data.masterIOCurrent = static_cast<float>(URCommon::getFloat(data, _messageOffset));

		// secret stuff, masterSafetyState, master
		_messageOffset+=2;

		//unsigned char euroMap = URCommon::getUChar(data, _messageOffset);
		URCommon::getUChar(data, _messageOffset);
	/*	if(euroMap==1){
			uint32_t it1 = euromapInputBits=getUINT32(_messageOffset);
			uint32_t it2 = data.euromapOutputBits=getUINT32(_messageOffset);
			uint16_t it3 = data.euromap24Voltage=getUINT16(_messageOffset);
			uint16_t it4 = data.euromap24Current=getUINT16(_messageOffset);
		}*/
		}
		break;

	case CARTESIAN_INFO:
		_messageOffset += packetLength-5;
		break;
	case LASER_POINTER_POSITION:
		_messageOffset += packetLength-5;
		break;
	default:
		_messageOffset += packetLength-5; //-5 because the header has already been read.
		//RW_WARN("Unknown package type: "<<(int)packetType<<"  "<<packetLength<<" MessageOffset = "<<_messageOffset<<std::endl);
	}
}


//Extract many booleans, max 16
bool URPrimaryInterface::extractBoolean(uint16_t input, unsigned int bitNumber) {
	uint16_t filter = static_cast<uint16_t>(1<<bitNumber);
	bool output = (input & filter)>0;
	return output;
}

