/* */
#ifndef RWHW_URPRIMARYINTERFACE_HPP
#define RWHW_URPRIMARYINTERFACE_HPP


/**
 * @file UniversalRobots.hpp
 */

#include "UniversalRobotsData.hpp"
#include "URMessage.hpp"

#include <rw/math/Q.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/task/Task.hpp>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread.hpp>

#include <queue>
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

	/**
	 * @brief Connect to the UR
	 *
	 * Throws a rw::common::Exception if failing to connect
	 * @param host [in] IP address of the UR
	 * @param port [in] Port for connecting to the UR. Defaults to 30002.
	 */
	void connect(const std::string& host, unsigned int port = 30002);

	/**
	 * @brief Returns the IP of the computer on which the application using the driver is running
	 */
	std::string getLocalIP();

	void start();
	void stop();

	bool sendScriptFile(const std::string& filename);
	bool sendScript(const std::string& script);

	bool isConnected() const;

	void disconnect();

	bool hasData() const;
	UniversalRobotsData getLastData() const;

	double driverTime() const;
	bool _lostConnection;
	long _lastPackageTime;

	std::queue<URMessage> getMessages() {
		return _messages;
	}

	void clearMessages() {
		while (_messages.empty() == false)
			_messages.pop();
	}

private:
	rw::common::Ptr<boost::thread> _thread;
	mutable boost::mutex _mutex;
	bool _stop;
	void run();


	bool readPrimaryInterfacePacket();
	void readRobotsState(const std::vector<char>& data);
	void readRobotMessage(const std::vector<char>& data, unsigned int messageLength);

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
	bool _hasURData;
	UniversalRobotsData _data;

	bool _lastTimeRunningProgram;

	std::vector<char> _dataStorage;

	static const unsigned char ROBOT_STATE = 16, ROBOT_MESSAGE = 20, HMC_MESSAGE = 22;
	static const unsigned char ROBOT_MODE_DATA = 0, JOINT_DATA = 1, TOOL_DATA = 2, MASTERBOARD_DATA = 3, CARTESIAN_INFO = 4, LASER_POINTER_POSITION = 5;
	static const unsigned char _commandNumberPrecision = 14;

	std::queue<URMessage> _messages;

};


} //end namespace

#endif //#ifndef RWHW_URPRIMARYINTERFACE_HPP
