/* */
#ifndef RWHW_URPRIMARYINTERFACE_HPP
#define RWHW_URPRIMARYINTERFACE_HPP


/**
 * @file UniversalRobots.hpp
 */

#include "UniversalRobotsData.hpp"

#include <rw/math/Q.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/task/Task.hpp>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread.hpp>


#include <fstream>




namespace rwhw {

/**
 * @brief Implements the interface for a UR
 */
class URPrimaryInterface  {

public:
	typedef rw::common::Ptr<URPrimaryInterface> Ptr;

	/**
	 * @brief Creates object
	 */
	URPrimaryInterface();

	~URPrimaryInterface();

	bool connect(const std::string& host, unsigned int port);

	void start();
	void stop();

	bool sendScript(const std::string& filename);

	bool isConnected() const;

	void disconnect();



	UniversalRobotsData getLastData() const;

private:
	rw::common::Ptr<boost::thread> _thread;
	boost::mutex _mutex;
	bool _stop;
	void run();


	bool readPrimaryInterfacePacket();
	void readRobotsState(uint32_t& messageOffset, uint32_t& messageLength);

	bool extractBoolean(uint16_t input, unsigned int bitNumber);
	bool sendCommand(const std::string &str);

	bool _haveReceivedSize;
	uint32_t messageLength, messageOffset;

	boost::asio::ip::tcp::socket* _socket;
	boost::asio::io_service _ioService;

	std::string _hostName;

	bool _connected;

	static const unsigned int max_buf_len = 5000000;
	char buf[max_buf_len];

	//Data
	UniversalRobotsData _data;

	bool _lastTimeRunningProgram;

	static const unsigned char ROBOT_STATE = 16, ROBOT_MESSAGE = 20, HMC_MESSAGE = 22;
	static const unsigned char ROBOT_MODE_DATA = 0, JOINT_DATA = 1, TOOL_DATA = 2, MASTERBOARD_DATA = 3, CARTESIAN_INFO = 4, LASER_POINTER_POSITION = 5;
	static const unsigned char _commandNumberPrecision = 14;
};


} //end namespace

#endif //#ifndef RWHW_URPRIMARYINTERFACE_HPP
