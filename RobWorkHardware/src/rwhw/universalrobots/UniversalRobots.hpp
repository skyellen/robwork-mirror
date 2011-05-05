/*
 * UniversalRobot.hpp
 *
 *  Created on: Apr 15, 2010
 *      Author: lpe
 */


#ifndef UNIVERSALROBOT_HPP
#define UNIVERSALROBOT_HPP
/**
 * @file UniversalRobot.hpp
 */

#include "UniversalRobotData.hpp"

#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/task/Task.hpp>

//#include <sandbox/VelocityRamps/SyncVelocityRamp.hpp>
// Boost

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>



#include <fstream>

typedef boost::uint16_t uint16;
typedef boost::uint32_t uint32;

	namespace boost { namespace asio {
		namespace ip { 
			class tcp; 
		} 
	} }

/**
 * @brief Implements the interface for a UR
 */
class UniversalRobot  {

public:
	typedef rw::common::Ptr<UniversalRobot> Ptr;

    /**
     * @brief Creates object
     */
    UniversalRobot();

    ~UniversalRobot();

	bool connect(const std::string& host, int port);
	void disconnect();
	bool isConnected() const;

	void update();
	bool moveTo(const rw::math::Q& q);
	bool executePath(const rw::trajectory::QPath& path);
	bool executeTrajectory(rwlibs::task::QTask::Ptr task, double speed, int id);


private:
	rw::models::Device::Ptr _device;
    rw::math::Q _sigma;
    rw::math::Q _tau;
    int _trajectoryId;


    //Socket
    bool connectSocket(const std::string &ip, unsigned int port);
    void disconnectSocket();

    void pathToScriptString(const rw::trajectory::QPath& path, std::ostringstream &stream);
    void qToScriptString(const rw::math::Q& q, int index, std::ostringstream &stream);
    bool servoJ(const rw::trajectory::QPath& path, const std::vector<double>& betas, double velScale, double accScale);
    bool moveJ(const rw::trajectory::QPath& path);
    bool readPacket();
    void readRobotState(uint32& messageOffset, uint32& messageLength);

	unsigned char getUchar(uint32 &messageOffset);
	uint16 getUINT16(uint32 &messageOffset);
	uint32 getUINT32(uint32 &messageOffset);
	float getFloat(uint32 &messageOffset);
	double getDouble(uint32 &messageOffset);
	long getLong(uint32 &messageOffset);
	bool getBoolean(uint32 &messageOffset);
	bool extractBoolean(uint16 input, unsigned int bitNumber);
	rw::math::Vector3D<double> getVector3D(uint32 &messageOffset);

	bool getChar(char* output);
	bool sendCommand(std::string &str);
	bool _haveReceivedSize;
	uint32 messageLength, messageOffset;
	boost::asio::ip::tcp::socket* _socket;
	boost::asio::io_service _io_service;
	std::string _hostName;
	unsigned int _hostPort;
	bool _connected;
	static const unsigned int max_buf_len = 5000000;
	char buf[max_buf_len];

	//Data
	UniversalRobotData _data;
	bool _lastTimeRunningProgram;

	static const unsigned char ROBOT_STATE = 16, ROBOT_MESSAGE = 20, HMC_MESSAGE = 22;
	static const unsigned char ROBOT_MODE_DATA = 0, JOINT_DATA = 1, TOOL_DATA = 2, MASTERBOARD_DATA = 3, CARTESIAN_INFO = 4, LASER_POINTER_POSITION = 5;
	static const unsigned char _commandNumberPrecision = 14;
};



#endif // end include guard
