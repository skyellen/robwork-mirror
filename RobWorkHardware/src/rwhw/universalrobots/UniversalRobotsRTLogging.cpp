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

#include "UniversalRobotsRTLogging.hpp"
#include "URCommon.hpp"

using namespace rwhw;
using namespace rw::math;
using namespace rw::common;

UniversalRobotsRTLogging::UniversalRobotsRTLogging():
	_socket(NULL),
   	_thread(NULL),
	_connected(false),
	_hasData(false),
	_lostConnection(false),
	_lastPackageTime(0),
	_stop(false)
{
}

UniversalRobotsRTLogging::~UniversalRobotsRTLogging() {
	disconnect();
}

double UniversalRobotsRTLogging::driverTime() {
	return URCommon::driverTimeStamp();
}

void UniversalRobotsRTLogging::start() {
	_stop = false;
	_thread = ownedPtr(new boost::thread(&UniversalRobotsRTLogging::run, this));
}

void UniversalRobotsRTLogging::stop() {
	_stop = true;
	if(_thread != NULL) {
      // Give the thread one second to stop
      if(!_thread->timed_join(boost::posix_time::seconds(1))) {
          // Failure, interrupt
          RW_WARN("Interrupting UniversalRobotsRTLogging receive thread...");
          _thread->interrupt();
          if(!_thread->timed_join(boost::posix_time::seconds(1)))
              RW_WARN("Failed to interrupt UniversalRobotsRTLogging receive thread");
      }
      _thread = NULL;
	}
}

void UniversalRobotsRTLogging::run() {
	while (!_stop) {
		if (readRTInterfacePacket())
			_hasData = true;

		// TODO: check when last package was received. If this is more than 100 milliseconds then
		// something bad probably happened
		if(_connected){
			long time = TimerUtil::currentTimeMs();
			if(time-_lastPackageTime>1000){
				_lostConnection = true;
			} else {
				_lostConnection = false;
			}

		}

		//_thread->yield();
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	}
}


void UniversalRobotsRTLogging::connect(const std::string& host, unsigned int port) {
	try {
		boost::asio::ip::tcp::resolver resolver(_ioService);
		boost::asio::ip::tcp::resolver::query query(host.c_str(), "");
		boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
		boost::asio::ip::tcp::endpoint ep = (*iter).endpoint();
		ep.port(port);
		_socket = new boost::asio::ip::tcp::socket(_ioService);
		_socket->connect(ep);
		_connected = true;
		_lastPackageTime = TimerUtil::currentTimeMs();
	} catch(boost::system::system_error& e) {
		RW_THROW("Unable to connect to command port with message: "<<e.what());
	}
}

void UniversalRobotsRTLogging::disconnect() {
	if(_socket != NULL) {
		_socket->shutdown(boost::asio::socket_base::shutdown_both);
		_socket->close();
		delete _socket;
		_connected = false;
	   _socket = NULL;
	}
}


URRTData UniversalRobotsRTLogging::getLastData() {
	boost::mutex::scoped_lock lock(_mutex);
	return _data;
}

bool UniversalRobotsRTLogging::hasData() const {
	return _hasData;
}

bool UniversalRobotsRTLogging::readRTInterfacePacket() {
	if(!_connected) {
		return false;
	}

    //Get the length of the available data
	size_t bytesReady = _socket->available();
	if (bytesReady < 4)
		return false;

	//If the first part of the packet is there, we will block until we have read the entire packet.

    unsigned int offset = 0;
    size_t msgSize = URCommon::getUInt32(_socket, offset);
    bytesReady = 0;
    int tries = 0;
    do {
		bytesReady = _socket->available();
	//	std::cout<<"-";
		tries += 1;

	} while (bytesReady < msgSize-4);



    double timestamp = driverTime();
    double time = URCommon::getDouble(_socket, offset);
    //std::cout<<"Time = "<<time<<std::endl;

    Q q_target = URCommon::getQ(_socket, 6, offset);
    Q dq_target = URCommon::getQ(_socket, 6, offset);
    Q ddq_target = URCommon::getQ(_socket, 6, offset);

    Q i_target = URCommon::getQ(_socket, 6, offset);
    Q m_target = URCommon::getQ(_socket, 6, offset);

    Q q_actual = URCommon::getQ(_socket, 6, offset);
    Q dq_actual = URCommon::getQ(_socket, 6, offset);
    Q i_actual = URCommon::getQ(_socket, 6, offset);

    Q acc_values = URCommon::getQ(_socket, 18, offset);

    Q tcp_force = URCommon::getQ(_socket, 6, offset);
    Q tool_pose = URCommon::getQ(_socket, 6, offset);
    Q tcp_speed = URCommon::getQ(_socket, 6, offset);

    uint64_t digin = URCommon::getUInt64(_socket, offset);
   // std::cout<<"Digital Inputs"<<digin<<std::endl;
    //unsigned int digin1 = URCommon::getUInt32(_socket, offset);
    //unsigned int digin2 = URCommon::getUInt32(_socket, offset);
    //std::cout<<"Offset = "<<offset<<std::endl;
    //Q temperatures = getQ(socket, 6, offset);

    //If there is any data left, just read it to empty buffer
    char* buffer = new char[msgSize];
    _socket->read_some(boost::asio::buffer(buffer, msgSize-offset));
    delete[] buffer;
    {
		boost::mutex::scoped_lock lock(_mutex);
		_data.driverTimeStamp = timestamp;
		_data.controllerTimeStamp = time;
		_data.qTarget = q_target;
		_data.dqTarget = dq_target;
		_data.ddqTarget = ddq_target;
		_data.iTarget = i_target;
		_data.torqueTarget = m_target;

		_data.qActual = q_actual;
		_data.dqActual = dq_actual;
		_data.iActual = i_actual;

		_data.accValues = acc_values;
		_data.tcpForce = tcp_force;
		_data.toolPose = tool_pose;
		_data.tcpSpeed = tcp_speed;
		_data.digIn = digin;
    }

    _lastPackageTime = TimerUtil::currentTimeMs();
    return true;




}
