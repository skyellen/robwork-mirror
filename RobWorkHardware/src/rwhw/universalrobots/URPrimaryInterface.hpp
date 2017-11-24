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

#ifndef RWHW_URPRIMARYINTERFACE_HPP
#define RWHW_URPRIMARYINTERFACE_HPP

#include "UniversalRobotsData.hpp"
#include "URMessage.hpp"

#include <rw/common/Ptr.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread.hpp>

#include <queue>

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

	/**
	 * @brief Destructor
	 */
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

	/**
	 * @brief Starts thread receiving data from the UR
	 */
	void start();

	/**
	 * @brief Stop the thread receiving data from the UR
	 *
	 * The method blocks until the thread has been stopped. Throws exception if unable to stop thread within a given timeout 
	 */
	void stop();

	/**
	 * @brief Send script file to the robot
	 * param filename [in] Path and filename of the ur script to send
	 * return true if succesfully sent
	 */
	bool sendScriptFile(const std::string& filename);

	/**
	* @brief Send script to the robot
	* @param script [in] Script to send
	* @return true if succesfully sent
	*/
	bool sendScript(const std::string& script);

	/**
	 * @brief Returns true if the robot is connected
	 * @return Returns true if the robot is connected. False otherwise. 
	 */
	bool isConnected() const;

	/** 
	 * @brief Disconnect the socket communication with the robot.
	 */
	void disconnect();

	/**
	 * @brief Returns true if data is available
	 */
	bool hasData() const;

	/** 
	 * @brief Returns the last retrived data
	 */
	UniversalRobotsData getLastData() const;

	/** 
	 * @brief Returns the current time according to the driver
	 */
	double driverTime() const;


	/**
	 * @brief Returns the URMessages received
	 */
	std::queue<URMessage> getMessages() {
		return _messages;
	}

	/**
	* @brief Clear the list of recieved URMessages
	*/
	void clearMessages() {
		while (_messages.empty() == false)
			_messages.pop();
	}

private:
	bool _lostConnection;
	long _lastPackageTime;


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
	uint32_t _messageLength, _messageOffset;

	boost::asio::ip::tcp::socket* _socket;
	boost::asio::io_service _ioService;

	std::string _hostName;

	bool _connected;


	//Data
	bool _hasURData;
	UniversalRobotsData _data;

	std::vector<char> _dataStorage;

	static const unsigned char ROBOT_STATE = 16, ROBOT_MESSAGE = 20, HMC_MESSAGE = 22;
	static const unsigned char ROBOT_MODE_DATA = 0, JOINT_DATA = 1, TOOL_DATA = 2, MASTERBOARD_DATA = 3, CARTESIAN_INFO = 4, LASER_POINTER_POSITION = 5;
	static const unsigned char _commandNumberPrecision = 14;

	std::queue<URMessage> _messages;

};


} //end namespace

#endif //#ifndef RWHW_URPRIMARYINTERFACE_HPP
