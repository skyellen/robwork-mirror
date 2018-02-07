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

#ifndef RWHW_UNIVERSALROBOTSRTLOGGING_HPP
#define RWHW_UNIVERSALROBOTSRTLOGGING_HPP

#include <rw/math/Q.hpp>
#include <rw/common/types.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

namespace rwhw {

/**
 * @brief Class for holding the log information provided by the UR on the 125Hz interface.
 *
 * The 125Hz interface is sometime referred to as the real-time interface, hence "RT" in the name
 */
class URRTData {
public:
    /**
	 * @brief Construct empty URRTData object
	 */
	URRTData(): driverTimeStamp(0), controllerTimeStamp(0), digIn(0)
	{

	}

	/**
	 * @brief Timestamp of when data arrived on the PC
	 */
	double driverTimeStamp;
	/**
	 * @brief Timestamp given to data by the UR controller
	 */
	double controllerTimeStamp;

	rw::math::Q qTarget;
	rw::math::Q dqTarget;
	rw::math::Q ddqTarget;
	rw::math::Q iTarget;
	rw::math::Q torqueTarget;

	rw::math::Q qActual;
	rw::math::Q dqActual;
	rw::math::Q iActual;

	rw::math::Q accValues;
	rw::math::Q tcpForce;
	rw::math::Q toolPose;
	rw::math::Q tcpSpeed;

	int64_t digIn;
};

class UniversalRobotsRTLogging {
	public:
	UniversalRobotsRTLogging();
	~UniversalRobotsRTLogging();

	void start();
	void stop();

	/**
	 * @brief Connects socket to UR on the real-time interface
	 *
	 * If not able to connect it throws a rw::common::Exception
	 * @param host [in] host address
	 * @param port [in] Port to connect to. Defaults to 30003.
	 */
	void connect(const std::string& host, unsigned int port = 30003);

	/**
	 * @brief Disconnects socket to UR on the real-time interface
	 */
	void disconnect();

	bool readRTInterfacePacket();

	bool hasData() const;
	URRTData getLastData();

	double driverTime();

	private:
	boost::asio::ip::tcp::socket* _socket;
	boost::asio::io_service _ioService;
	rw::common::Ptr<boost::thread> _thread;
	boost::mutex _mutex;
	bool _connected;
	bool _hasData;
	bool _lostConnection;
	long _lastPackageTime;
	bool _stop;
	void run();

	URRTData _data;
};

} //end namespace

#endif //#ifndef RWHW_UNIVERSALROBOTSRTLOGGING_HPP
