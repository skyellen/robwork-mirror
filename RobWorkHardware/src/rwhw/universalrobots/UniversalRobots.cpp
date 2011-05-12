/*
 * UniversalRobots.cpp
 *
 *  Created on: Apr 15, 2010
 *      Author: lpe
 */

#include "UniversalRobots.hpp"



#include <rw/common/TimerUtil.hpp>

#include <iostream>
#include <string>
#include <float.h>




using namespace rw::models;
using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;

using namespace rwhw;
using namespace boost::asio::ip;



UniversalRobots::UniversalRobots():
	_haveReceivedSize(false),
	_socketPrimary(0),
	_socketControl(0),
	_socketRTInterface(0),
	_connectedPrimary(false),
	_connectedControl(false),
	_connectedRTInterface(false)
{
}

UniversalRobots::~UniversalRobots() {
	disconnect();
}

bool UniversalRobots::isConnectedPrimary() const {
	return _connectedPrimary;
}

bool UniversalRobots::isConnectedControl() const {
	return _connectedControl;
}


bool UniversalRobots::isConnectedRTInterface() const {
	return _connectedRTInterface;
}

bool UniversalRobots::connectPrimary(const std::string& host, unsigned int port) {
	if (_connectedPrimary) {
		RW_THROW("Already connected. Disconnect before connecting again!");

	}

	_socketPrimary = connectSocket(host, port, _ioServicePrimary);
	if (_socketPrimary == NULL) {
		_connectedPrimary = false;
		return false;
	}
	_connectedPrimary = true;
    return true;
}

bool UniversalRobots::connectRTInterface(const std::string& host, unsigned int port) {
	if (_connectedRTInterface) {
		RW_THROW("Already connected. Disconnect before connecting again!");

	}
	_socketRTInterface = connectSocket(host, port, _ioServiceRTInterface);
	if (_socketRTInterface == NULL) {
		_connectedRTInterface = false;
		return false;
	}
	_connectedRTInterface = true;
    return true;
}


bool UniversalRobots::sendScript(const std::string& filename)
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

	return sendCommand(_socketPrimary, std::string(buffer));
}


void UniversalRobots::disconnect() {
	disconnectPrimary();
	disconnectRTInterface();
	disconnectControl();

}

void UniversalRobots::disconnectPrimary() {
	disconnectSocket(_socketPrimary);
}

void UniversalRobots::disconnectControl() {
	disconnectSocket(_socketControl);
}


void UniversalRobots::disconnectRTInterface() {
	disconnectSocket(_socketRTInterface);
}


void UniversalRobots::update() {

	//readRTInterfacePackage(_statusSocket);
	//while(readPacket(_statusSocket));
}


bool UniversalRobots::moveTo(const rw::math::Q& q) {
	QPath path;
	path.push_back(q);
	return moveJ(path);
}
	
	


bool UniversalRobots::executePath(const rw::trajectory::QPath& path) {
	//bool success = true;
	BOOST_FOREACH(const Q& q, path) {
		if (!moveTo(q))
			return false;
	}
	return true;
}
    


bool UniversalRobots::executeTrajectory(rwlibs::task::QTask::Ptr task, double speed, int id) {
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
tcp::socket* UniversalRobots::connectSocket(const std::string &ip, unsigned int port, boost::asio::io_service& ioService) {
	try {
		boost::asio::ip::tcp::resolver resolver(ioService);
		boost::asio::ip::tcp::resolver::query query(ip.c_str(), "");
		boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
		boost::asio::ip::tcp::endpoint ep = (*iter).endpoint();
		ep.port(port);
		//Connecting to server
		tcp::socket* socket = new boost::asio::ip::tcp::socket(ioService);
		socket->connect(ep);
		return socket;
	} catch(boost::system::system_error& e) {
		RW_THROW("Unable to connect to command port with message: "<<e.what());
	}
}

//Disconnect the socket
void UniversalRobots::disconnectSocket(tcp::socket*& socket) {
	if(socket != NULL) {
		socket->shutdown(boost::asio::socket_base::shutdown_both);
		socket->close();
		delete socket;
	}
	socket = NULL;
}

bool UniversalRobots::getChar(tcp::socket* socket, char* output) {
	return socket->read_some(boost::asio::buffer(output, 1));
}

//Send the script to the robot
bool UniversalRobots::sendCommand(tcp::socket* socket, const std::string &str) {
	if(!_connectedPrimary) {
//		log(Error)<< "Can not send command to " << _device->getName() << ", socket not connected" <<endlog();
		return false;
	}

//	log(Info) << "script: " << str << endlog();
	socket->send(boost::asio::buffer(str.c_str(), str.size()));
	return true;
}

void UniversalRobots::pathToScriptString(const rw::trajectory::QPath& path, std::ostringstream &stream) {
	stream << "[";
	for(size_t i = 0; i < path.size()-1; i++) {
		for(size_t j = 0; j < path[i].size(); j++) {
			stream << path[i][j] << ",";
		}
	}
	for(size_t j = 0; j < path[path.size()-1].size()-1; j++) {
		stream << path[path.size()-1][j] << ",";
	}
	stream << path[path.size()-1][path[path.size()-1].size()-1] << "]";
}

// Parse the rw::math::Q as a string suitable for scripts
void UniversalRobots::qToScriptString(const rw::math::Q& q, int index, std::ostringstream &stream) {
	stream << "q" << index << "=[";
	for(size_t i = 0; i < q.size() - 1; i++) {
		stream << q[i] << ", ";
	}
	stream << q[q.size() - 1] << "]";
}

// Move linearly in joint space along a path
bool UniversalRobots::servoJ(const rw::trajectory::QPath& path, const std::vector<double>& betas, double velScale, double accScale) {
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
	return sendCommand(_socketPrimary, out);
}

/*
bool UniversalRobots::moveJ(const rw::math::Transform3D<>& transform) {
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
bool UniversalRobots::moveJ(const rw::trajectory::QPath& path) {
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
	return sendCommand(_socketPrimary, out);
}

bool UniversalRobots::readRTInterfacePacket(tcp::socket* socket) {

    //Get the length of the available data
	uint32 bytesReady = 0;
	do {
		bytesReady = socket->available();
	} while (bytesReady < 4);

    unsigned int offset = 0;
    size_t msgSize = getUINT32(socket, offset);
    bytesReady = 0;
    do {
		bytesReady = socket->available();
	} while (bytesReady < msgSize-4);

    std::cout<<"Other message size = "<<msgSize<<std::endl;


    double time = getDouble(socket, offset);
    std::cout<<"Time = "<<time<<std::endl;

    Q q_target = getQ(socket, 6, offset);
    Q dq_target = getQ(socket, 6, offset);
    Q ddq_target = getQ(socket, 6, offset);

    Q i_target = getQ(socket, 6, offset);
    Q m_target = getQ(socket, 6, offset);

    Q q_actual = getQ(socket, 6, offset);
    std::cout<<"q = "<<q_actual<<std::endl;
    Q dq_actual = getQ(socket, 6, offset);
    Q i_actual = getQ(socket, 6, offset);

    Q acc_values = getQ(socket, 18, offset);

    Q tcp_force = getQ(socket, 6, offset);
    Q tool_pose = getQ(socket, 6, offset);
    Q tcp_speed = getQ(socket, 6, offset);

    unsigned int digin1 = getUINT32(socket, offset);
    unsigned int digin2 = getUINT32(socket, offset);
    //std::cout<<"Offset = "<<offset<<std::endl;
    //Q temperatures = getQ(socket, 6, offset);


 //   std::cout<<"q = "<<q<<std::endl;
    char* buffer = new char[msgSize];
    socket->read_some(boost::asio::buffer(buffer, msgSize-offset));
    /*std::cout<<"Chars "<<(unsigned int)buffer[0]<<" "<<(unsigned int)buffer[1]<<" "<<(unsigned int)buffer[2]<<" "<<(unsigned int)buffer[3]<<std::endl;
    std::cout<<"Size of int = "<<sizeof(int)<<std::endl;
    unsigned int msgSize1 = *(int*)(&buffer[0]);
    unsigned int msgSize2 = ((int*)buffer)[0];
    unsigned int msgSize3 = ((int*)(&buffer[0]))[0];
    std::cout<<"msgSize1 = "<<msgSize1<<std::endl;
    std::cout<<"msgSize2 = "<<msgSize2<<std::endl;
    std::cout<<"msgSize3 = "<<msgSize3<<std::endl;
    double time = *(double*)(&buffer[4]);
    */


	//std::cout<<"Time = "<<time<<std::endl;

    //buffer[bytesReady] = 0;
    //std::cout<<"Read: "<<buffer<<std::endl;
    return true;


}

//Read incomming data
bool UniversalRobots::readPacketPrimaryInterface(tcp::socket* socket) {
	if(!_connectedPrimary) {
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
    uint32 bytesReady = socket->available();
    std::cout<<"bytes ready "<<bytesReady<<std::endl;
	//Check if the data can contain an valid messages length
	if(bytesReady < (uint32)sizeof(uint32))
		 return false; //Wait for a who int are ready

	//Get the message length
	if(!_haveReceivedSize) {
		messageLength = getUINT32(socket, messageOffset);
		_haveReceivedSize = true;
	}

	//Check if the data contain the who messages
	std::cout<<"Compares "<<bytesReady<<" < "<<messageLength<<std::endl;
	if (bytesReady < messageLength)
		return false; //Wait for a who packet are ready

	//Receive the meassage type
	unsigned char messageType = getUchar(socket, messageOffset);

	//Do until the who messages are analysed
	while(messageOffset < messageLength) {
		switch(messageType)
		{
			//Analyse the robot state
			case ROBOT_STATE:
					readRobotsState(socket, messageOffset, messageLength);
				break;

			//Flush the other messages types, as they not yet have any interest for this protocol
			case ROBOT_MESSAGE:
			case HMC_MESSAGE:
			default:
				getUchar(socket, messageOffset);

			break;
		}
	}
	_haveReceivedSize=false;
	return true;
}

//Read the robot start
void UniversalRobots::readRobotsState(tcp::socket* socket, uint32& messageOffset, uint32& messageLength) {
	//Temp variabels
	rw::math::Q jointPosition(6);
	rw::math::Q targetJointPosition(6);
	rw::math::Q jointSpeed(6);
	uint16 tmp16;
	//bool _lastTimeRunningProgram= _data.programRunning;

	//Do until the who messages are analysed
	while(messageOffset<messageLength) 	{
		//Get the packet length
		uint16 packetLength=getUINT32(socket, messageOffset);
		//Get the packet type
		unsigned char packetType=getUchar(socket, messageOffset);
		switch(packetType) {
		case ROBOT_MODE_DATA:
			//long TimeStamp
			_data.timestamp = getLong(socket, messageOffset);
			std::cout<<"Time Stamp = "<<_data.timestamp<<std::endl;
			//bool physicalRobotsConnected
			_data.physical = getBoolean(socket, messageOffset);

			// bool realRobotsEnabled
			_data.real = getBoolean(socket, messageOffset);

			// bool robot_power_on
			_data.robotPowerOn = getBoolean(socket, messageOffset);
			std::cout<<"Power On = "<<_data.robotPowerOn<<std::endl;

			// bool emergency_stopped
			_data.emergencyStopped = getBoolean(socket, messageOffset);
			std::cout<<"Emergency Stopped = "<<_data.emergencyStopped<<std::endl;
			// bool security_stopped
			_data.securityStopped = getBoolean(socket, messageOffset);
			std::cout<<"Security Stopped = "<<_data.securityStopped<<std::endl;

			// bool program_running
			_data.programRunning = getBoolean(socket, messageOffset);

			// bool program_paused
			_data.programPaused = getBoolean(socket, messageOffset);

			// unsigned char robotMode
//			unsigned char mode = getUchar(messageOffset);
//			if (mode < 0) {
//				mode += 256;
//			}
			_data.robotMode = getUchar(socket, messageOffset);

			//double speedFraction
			_data.speedFraction = getDouble(socket, messageOffset);

			break;

		case JOINT_DATA:
			//For all 6 joints
			for (unsigned int j = 0; j < 6; j++) {
				//get rw::math::Q
				jointPosition[j] = getDouble(socket, messageOffset);
				targetJointPosition[j] = getDouble(socket, messageOffset);
				jointSpeed[j] = getDouble(socket, messageOffset);

				//store Array
				_data.jointCurrent[j] = getFloat(socket, messageOffset);
				_data.jointVoltage[j] = getFloat(socket, messageOffset);
				_data.jointMotorTemperature[j] = getFloat(socket, messageOffset);
				_data.jointMicroTemperature[j] = getFloat(socket, messageOffset);
				_data.jointMode[j] = getUchar(socket, messageOffset);
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
			_data.analogInputRange[2] = getUchar(socket, messageOffset);
			_data.analogInputRange[3] = getUchar(socket, messageOffset);

			// double Analog input
			_data.analogIn[2] = getDouble(socket, messageOffset);
			_data.analogIn[3] = getDouble(socket, messageOffset);

			// float toolVoltage48V
			_data.toolVoltage48V = getFloat(socket, messageOffset);

			// unsigned char tool_output_voltage (can only be 0, 12 or 24)
			_data.toolOutputVoltage = getUchar(socket, messageOffset);

			// float tool_current
			_data.toolCurrent = getFloat(socket, messageOffset);

			// float toolTemperature;
			_data.toolTemperature = getFloat(socket, messageOffset);

			// uchar tool mode
			_data.toolMode = getUchar(socket, messageOffset);
			break;

		case MASTERBOARD_DATA:
			// unsigned short digitalInputBits;
			tmp16 = getUINT16(socket, messageOffset);
			for(unsigned int i=0; i<10; ++i) // Tool bits are actually included here
				_data.digitalIn[i] = extractBoolean(tmp16, i);

			// unsigned short digitalOutputBits;
			tmp16 = getUINT16(socket, messageOffset);
			for(unsigned int i=0; i<10; ++i) // Tool bits are actually included here
				_data.digitalOut[i] = extractBoolean(tmp16, i);

			// unsigned char analogInputRange[4]
			_data.analogInputRange[0] = getUchar(socket, messageOffset);
			_data.analogInputRange[1] = getUchar(socket, messageOffset);

			//double analogInput[4];
			_data.analogIn[0] = getDouble(socket, messageOffset);
			_data.analogIn[1] = getDouble(socket, messageOffset);

			// char analogOutputDomain[2]
			_data.analogOutputDomain[0] = getUchar(socket, messageOffset);
			_data.analogOutputDomain[1] = getUchar(socket, messageOffset);

			//double analogOutput[2];
			_data.analogOut[0] = getDouble(socket, messageOffset);
			_data.analogOut[1] = getDouble(socket, messageOffset);

			// float masterTemperature;
			_data.masterTemperature = getFloat(socket, messageOffset);

			// float robotVoltage48V;
			_data.robotVoltage48V = getFloat(socket, messageOffset);

			// float robotCurrent;
			_data.robotCurrent = getFloat(socket, messageOffset);

			// float masterIOCurrent;
			_data.masterIOCurrent = getFloat(socket, messageOffset);
			break;

		case CARTESIAN_INFO:
			_data.toolPosition = getVector3D(socket, messageOffset);
			_data.toolAxixAngle = getVector3D(socket, messageOffset);
			break;

		case LASER_POINTER_POSITION:
			_data.laserPointerPosition = getVector3D(socket, messageOffset);
			break;
		default:
			for(unsigned int i = 5; i<packetLength; i++)
				getUchar(socket, messageOffset);
			break;
		}
	}

}



//Extract a unsigned char
unsigned char UniversalRobots::getUchar(tcp::socket* socket, uint32 &messageOffset) {
	unsigned char output = 0;
	getChar(socket, (char*)&output+0);
	messageOffset += 1;
	return output;
}

//Extract a 16 bit unsigned int
uint16 UniversalRobots::getUINT16(tcp::socket* socket, uint32 &messageOffset) {
	uint16 output = 0;
	getChar(socket, (char*)&output+1);
	getChar(socket, (char*)&output+0);
	messageOffset += 2;
	return output;
}

//Extract a 32 bit unsigned int
uint32 UniversalRobots::getUINT32(tcp::socket* socket, uint32 &messageOffset) {
	uint32 output = 0;
	getChar(socket, (char*)&output+3);
	getChar(socket, (char*)&output+2);
	getChar(socket, (char*)&output+1);
	getChar(socket, (char*)&output+0);
	messageOffset += 4;
	return output;
}

//Extract a float
float UniversalRobots::getFloat(tcp::socket* socket, uint32 &messageOffset) {
	float output = 0;
	getChar(socket, (char*)&output+3);
	getChar(socket, (char*)&output+2);
	getChar(socket, (char*)&output+1);
	getChar(socket, (char*)&output+0);
	messageOffset += 4;
	return output;
}

//Extract a double
double UniversalRobots::getDouble(tcp::socket* socket, uint32 &messageOffset) {
	double output = 0;
	getChar(socket, (char*)&output+7);
	getChar(socket, (char*)&output+6);
	getChar(socket, (char*)&output+5);
	getChar(socket, (char*)&output+4);
	getChar(socket, (char*)&output+3);
	getChar(socket, (char*)&output+2);
	getChar(socket, (char*)&output+1);
	getChar(socket, (char*)&output+0);
	messageOffset += 8;
	return output;
}

//Extract a Long
long UniversalRobots::getLong(tcp::socket* socket, uint32 &messageOffset) {
	long output = 0;
	getChar(socket, (char*)&output+7);
	getChar(socket, (char*)&output+6);
	getChar(socket, (char*)&output+5);
	getChar(socket, (char*)&output+4);
	getChar(socket, (char*)&output+3);
	getChar(socket, (char*)&output+2);
	getChar(socket, (char*)&output+1);
	getChar(socket, (char*)&output+0);
	messageOffset += 8;
	return output;
}

//Extract a one boolean
bool UniversalRobots::getBoolean(tcp::socket* socket, uint32 &messageOffset) {
	char tmp = 0;
	getChar(socket, (char*)&tmp);
	bool output = (tmp & 1)>0;
	messageOffset += 1;
	return output;
}

//Extract many booleans, max 16
bool UniversalRobots::extractBoolean(uint16 input, unsigned int bitNumber) {
	uint16 filter = 1<<bitNumber;
	bool output = (input & filter)>0;
	return output;
}

//Extract a 3d vector
rw::math::Vector3D<double> UniversalRobots::getVector3D(tcp::socket* socket, uint32 &messageOffset) {
	rw::math::Vector3D<double> output;
	output[0] = getDouble(socket, messageOffset);
	output[1] = getDouble(socket, messageOffset);
	output[2] = getDouble(socket, messageOffset);
	return output;
}

Q UniversalRobots::getQ(tcp::socket* socket, int cnt, uint& messageOffset) {
	Q res(cnt);
	for (size_t i = 0; i<cnt;i++) {
		res(i) = getDouble(socket, messageOffset);
	}
	return res;
}
