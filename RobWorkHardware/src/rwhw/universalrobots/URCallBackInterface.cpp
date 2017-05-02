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

#include "URCallBackInterface.hpp"
#include "URCommon.hpp"
#include <rw/math/Transform3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/common/Log.hpp>
#include <fstream>

#include "urscript.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwhw; 
using namespace boost::asio::ip;

namespace {
	const double FLOAT_TO_INT_SCALE = 10000;
    const double NORMALIZE_PERCENTAGE = 0.01;
}

URCallBackInterface::URCallBackInterface():
    _cb(CB3),
    _thread(NULL),
    _callbackPort(33334),
    _stopServer(false),
    _robotStopped(true),
    _isMoving(false),
    _isServoing(false)
{
    /* Set the global loglevel to debug */
    Log::log().setLevel(Log::Debug);
}

bool URCallBackInterface::isMoving() const {
    return _isMoving;
}

double URCallBackInterface::driverTime() const {
    return URCommon::driverTimeStamp();
}

void URCallBackInterface::connect(const std::string& host, const unsigned int port) {
    _urPrimary.connect(host, port);
}

/* Deprecated function */
//void URCallBackInterface::startInterface(const unsigned int callbackPort, const std::string& filename) {
//    startCommunication("192.168.100.1", callbackPort, filename);
//}

void URCallBackInterface::startCommunication(const unsigned int callbackPort, ControllerBox cb,  const std::string& filename) {
	startCommunication(_urPrimary.getLocalIP(), callbackPort, cb, filename);
}

void URCallBackInterface::startCommunication(const std::string& callbackIP, const unsigned int callbackPort, ControllerBox cb, const std::string& filename) {
    _cb = cb;
	_callbackPort = callbackPort;
    /* There should be some better error checking at the conversion of the IP from string to ip::address - but the documentation is limited with what error codes that can be returned / exception(s) that can be thrown - [look at the source for more specific error documentation/information] */
    _callbackIP = boost::asio::ip::address::from_string(callbackIP);

    // launch communication thread
    RW_LOG_DEBUG("Spawning communication thread");
    _thread = ownedPtr(new boost::thread(boost::bind(&URCallBackInterface::run, this)));

    // load UR script
    std::string script;
    if (!filename.empty()) { // load UR script from file
		RW_LOG_INFO("Loading UR script from the file: " << filename);
        std::ifstream infile(filename.c_str());
        RW_LOG_INFO("Script loaded.");
        // get length of file:
        infile.seekg (0, std::ios::end);
        long long length = infile.tellg();
        infile.seekg (0, std::ios::beg);

        // allocate memory:
        char *buffer = new char [length+1];

        // read data as a block:
        infile.read(buffer, length);
        buffer[length] = '\0';
		
        script = std::string(buffer);
        delete[] buffer;
    } else { // use default script
        RW_LOG_DEBUG("Using default UR script/file");
        script = UR_SCRIPT;
    }
	
    // replace PORT placeholder with correct port value
    std::stringstream sstr_port;
    std::size_t n = script.find("PORT");
    if (n == std::string::npos) {
        RW_WARN("Unable to find PORT in script");
        sstr_port << script;
    } else {
        sstr_port << script.substr(0, n) << callbackPort << script.substr(n + 4);
    }
    script = sstr_port.str();
	
    // replace HOST placeholder with correct port value
    std::stringstream sstr_host;
    n = script.find("HOST");
    if (n == std::string::npos) {
        RW_WARN("Unable to find HOST in script");
        sstr_host << script;
    } else {
        std::stringstream sstr;
        RW_ASSERT(_callbackIP.is_v4());
        sstr_host << script.substr(0, n) << "\"" << _callbackIP.to_string() << "\"" << script.substr(n + 4);
    }
    script = sstr_host.str();
	
	// Remove commands not fitting for the specific version of the controller
	
	
	std::string cbId;
	switch (cb) {
	case CB2:
		cbId = "CB2";
		break;
	case CB3:
		cbId = "CB3";
		break;
	default:
		RW_THROW("Unknown controller box specified: "<<cb);
		break;
	}

	n = script.find("$");
	while (n != std::string::npos) {	
		if (script.substr(n+1,3) != cbId) {			
			script.erase(n, script.find("\n", n)-n+1);
		} else {
			script.erase(n, 4);
		}
		n = script.find("$");
	}

    // send script to robot
    RW_LOG_DEBUG("Sending script to the robot");
    bool sendScriptStatus = false;
    sendScriptStatus = _urPrimary.sendScript(script);
    if (sendScriptStatus) {
        RW_LOG_DEBUG("Successfully sent the script to the robot");
    } else {
        RW_LOG_ERROR("Was not successful in sending the script to the robot.");
    }
    _urPrimary.start();
}



void URCallBackInterface::stopCommunication() {
    _stopServer = true;
}


URPrimaryInterface& URCallBackInterface::getPrimaryInterface() {
    return _urPrimary;
}


namespace {

    void q2intVector(const Q& q, std::vector<int>& integers, int offset) {
		RW_ASSERT(offset+q.size()-1 < integers.size());
        for (size_t i = 0; i<q.size(); i++) {
            integers[offset+i] = static_cast<int>(q(i)*FLOAT_TO_INT_SCALE);
        }
    }

    void t2intVector(const Transform3D<>& transform, std::vector<int>& integers, int offset) {
		RW_ASSERT(offset+5 < integers.size());
		const Vector3D<>& p = transform.P();
		EAA<> eaa(transform.R());
    
        integers[offset] = p(0)*FLOAT_TO_INT_SCALE;
        integers[offset+1] = p(1)*FLOAT_TO_INT_SCALE;
        integers[offset+2] = p(2)*FLOAT_TO_INT_SCALE;
        integers[offset+3] = eaa(0)*FLOAT_TO_INT_SCALE;
        integers[offset+4] = eaa(1)*FLOAT_TO_INT_SCALE;
        integers[offset+5] = eaa(2)*FLOAT_TO_INT_SCALE;

    }


    void vector3d2intVector(const Vector3D<>& vec, std::vector<int>& integers, int offset) {
	RW_ASSERT(offset+2 < integers.size());
	integers[offset] = vec[0]*FLOAT_TO_INT_SCALE;
	integers[offset+1] = vec[1]*FLOAT_TO_INT_SCALE;
	integers[offset+2] = vec[2]*FLOAT_TO_INT_SCALE;
    }


    void wrench2intVector(const Wrench6D<>& wrench, std::vector<int>& integers, int offset) {
	RW_ASSERT(offset+5 < integers.size());
	integers[offset] = wrench.force()(0)*FLOAT_TO_INT_SCALE;
	integers[offset+1] = wrench.force()(1)*FLOAT_TO_INT_SCALE;
	integers[offset+2] = wrench.force()(2)*FLOAT_TO_INT_SCALE;
	integers[offset+3] = wrench.torque()(0)*FLOAT_TO_INT_SCALE;
	integers[offset+4] = wrench.torque()(1)*FLOAT_TO_INT_SCALE;
	integers[offset+5] = wrench.torque()(2)*FLOAT_TO_INT_SCALE;
    }

}


void URCallBackInterface::sendStop(tcp::socket& socket) {
    std::vector<int> integers(9);
    integers[0] = URScriptCommand::STOP;
    URCommon::send(&socket, integers);    
}
 
void URCallBackInterface::handleCmdRequest(tcp::socket& socket) {
    //RW_LOG_DEBUG("Handling command request - obtaining lock...");
    boost::mutex::scoped_lock lock(_mutex);
    //RW_LOG_DEBUG("Obtained lock");

    std::vector<int> integers(9);

    if (_commands.size() == 0 || (_isMoving && !_isServoing)) {		
        integers[0] = URScriptCommand::DO_NOTHING;
        URCommon::send(&socket, integers);
        // RW_LOG_DEBUG("Do nothing");
        return;
    }

	//std::cout<<"Handle Cmd Request ="<<_commands.size()<<std::endl;

    URScriptCommand cmd = _commands.front();
    //RW_LOG_DEBUG("Command type: " << cmd._type);
    _isServoing = false;
    integers[0] = cmd._type;
    switch (cmd._type) {
    case URScriptCommand::STOP:
        break;
    case URScriptCommand::MOVEQ:
        q2intVector(cmd._q, integers, 1);
        integers[7] = (cmd._speed*NORMALIZE_PERCENTAGE)*FLOAT_TO_INT_SCALE;
        integers[8] = (cmd._blend)*FLOAT_TO_INT_SCALE;
        _isMoving = true;
        break;
    case URScriptCommand::MOVET: 
        t2intVector(cmd._transform, integers, 1);
        integers[7] = (cmd._speed*NORMALIZE_PERCENTAGE)*FLOAT_TO_INT_SCALE;
        integers[8] = (cmd._blend)*FLOAT_TO_INT_SCALE;
        _isMoving = true;
        break;
    case URScriptCommand::SERVOQ:
        // make sure that q is not too big... eg. it should be reachable within 0.008 seconds
        q2intVector(cmd._q, integers, 1);
        _isMoving = true;
        _isServoing = true;
        break;
    case URScriptCommand::FORCE_MODE_START:
        integers.resize(26);
        t2intVector(cmd._transform, integers, 1);
        integers[7] = 0;
        q2intVector(cmd._selection, integers, 8);
        wrench2intVector(cmd._wrench, integers, 14);
        q2intVector(cmd._limits, integers, 20);
        //_isMoving = true;
        break;
    case URScriptCommand::FORCE_MODE_UPDATE:
        wrench2intVector(cmd._wrench, integers, 1);
        integers[7] = 0;
        //_isMoving = true;
        break;
    case URScriptCommand::FORCE_MODE_END:
        break;
	case URScriptCommand::TEACH_MODE_START:
		break;
	case URScriptCommand::TEACH_MODE_END:
		break;
	case URScriptCommand::SET_DIGOUT:
		integers[1] = cmd._id;
		integers[2] = cmd._bValue;
		break;
    case URScriptCommand::SET_PAYLOAD:
		integers[1] = cmd._mass*FLOAT_TO_INT_SCALE;
		vector3d2intVector(cmd._centerOfGravity, integers, 2);
        break;    
	default:
        RW_LOG_DEBUG("Unsupported command: " << cmd._type);
        RW_THROW("Unsupported command type: " << cmd._type);
        break;
    }
    //RW_LOG_DEBUG("Sending command to robot...");
    //std::cout<<"Speed="<<integers.at(7)<<" Blend="<<integers.at(8)<<std::endl;
    URCommon::send(&socket, integers);
    //RW_LOG_DEBUG("Command to robot sent");

    if (cmd._type != URScriptCommand::SERVOQ) {                
        RW_LOG_DEBUG("Popping commands as the command type was not SERVOQ");
        _commands.pop();
    }
}

void URCallBackInterface::run() {
    try
    {
	boost::asio::io_service io_service;
	RW_LOG_DEBUG("Callback IP = "<<_callbackIP);
	RW_LOG_DEBUG("Callback Port = "<<_callbackPort);
	tcp::acceptor acceptor(io_service, tcp::endpoint(_callbackIP, _callbackPort));
	while(!_stopServer)
	{
            tcp::socket socket(io_service);
			
            //std::cout<<"Ready to accept incoming connections "<<std::endl;
            RW_LOG_DEBUG("Ready to accept incoming connections on port '" << _callbackPort << "'");
            acceptor.accept(socket);
            RW_LOG_DEBUG("Incoming connection accepted");
            //std::cout<<"Incoming accepted"<<std::endl;
            //Timer timer;
			//Timer timer2;
            //timer.resetAndResume();  
			//Statistics<double> stats;
			//int cnt = 0;
            while (!_stopServer) {
			//  std::cout<<"\b\b\b\b\bm = "<<_isMoving;
//          std::cout<<"Time = "<<TimerUtil::currentTimeUs()<<std::endl; 
                boost::system::error_code error;
                size_t available = socket.available(error);
                if (available == 0) {
                    if (error == boost::asio::error::eof) {
                        //std::cout<<"Reached EOF"<<std::endl;
                        RW_LOG_DEBUG("Reached EOF on the socket.");
                        break;
                    } else {
#if 0
/* Commented out due to missing error code to catch the case where there are no errors, but 0 bytes are available to be read - could catch a posix specific 0=successful error code from the boost system error codes, but that seem a bit too hackish... */
                        /* Unknown error occured */
                        RW_LOG_ERROR("Unkown error '" << error << "' occured when querying for available bytes on the socket.");
                        /* Just jump up a level, to have the socket/connection be reestablished */
                        break;
#endif
                    }
                } else {

					char ch;
                    if (!URCommon::getChar(&socket, &ch))
                        continue;

					//stats.add(timer2.getTimeMs());
					//std::cout<<"\b\b\b\b\b\b\b T="<<timer2.getTimeMs()<<std::endl;
					//timer2.resetAndResume();

               
                    //RW_LOG_DEBUG("Received the byte/char '" << ch << "' from the connection.");

                    if (_robotStopped) {
                        //RW_LOG_DEBUG("Robot is stopped, going to send stop.");
                        sendStop(socket);
                    } else {
                        if (ch == 0) {
                            _isMoving = false;
                        } 
                        //RW_LOG_DEBUG("Going to handle command request - Time: " << timer.getTime());
                        //timer.resetAndResume();
                        handleCmdRequest(socket);
                    }

					//if (cnt++ % 100 == 0) {
					//	std::cout<<"Avg/min/max/stddev Time = "<<stats.mean()<<"/"<<stats.minValue()<<"/"<<stats.maxValue()<<"/"<<std::sqrt(stats.variance())<<std::endl;
					//	stats.clear();
					//}

                }
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
	}
    }
    catch (std::exception& e)
    {
		std::cerr << e.what() << std::endl;
    }
}


void URCallBackInterface::stopRobot() {
	RW_LOG_DEBUG("rwhw::URCallBackInterface::stopRobot()");
    //std::cout<<"RWHW Stop UR"<<std::endl;
    //_commands.push(URScriptCommand(URScriptCommand::STOP));
    boost::mutex::scoped_lock lock(_mutex);
    _robotStopped = true;
    _isMoving = false;
    while (!_commands.empty())
        _commands.pop();
}

void URCallBackInterface::popAllUpdateCommands() {
	
    while ((_commands.size() > 0) &&
           ((_commands.front()._type == URScriptCommand::SERVOQ) || (_commands.front()._type == URScriptCommand::FORCE_MODE_UPDATE)))
    {
        _commands.pop();
    }
}

void URCallBackInterface::moveQ(const rw::math::Q& q, float speed, float blend) {
    RW_LOG_DEBUG("rwhw::URCallBackInterface::moveQ("<<q<<","<<speed<<","<<blend<<")");
    boost::mutex::scoped_lock lock(_mutex);
    
    popAllUpdateCommands();

    _commands.push(URScriptCommand(URScriptCommand::MOVEQ, q, speed, blend));
    _robotStopped = false;
}

void URCallBackInterface::moveQ(const rw::math::Q& q, float speed) {
  RW_LOG_DEBUG("rwhw::URCallBackInterface::moveQ("<<q<<","<<speed<<")");
  boost::mutex::scoped_lock lock(_mutex);

  popAllUpdateCommands();

  _commands.push(URScriptCommand(URScriptCommand::MOVEQ, q, speed));
  _robotStopped = false;
}

void URCallBackInterface::moveT(const rw::math::Transform3D<>& transform, float speed, float blend) {
    RW_LOG_DEBUG("rwhw::URCallBackInterface::moveT("<<transform<<","<<speed<<","<<blend<<")");
    boost::mutex::scoped_lock lock(_mutex);

    popAllUpdateCommands();

    _commands.push(URScriptCommand(URScriptCommand::MOVET, transform, speed, blend));
    _robotStopped = false;
}

void URCallBackInterface::moveT(const rw::math::Transform3D<>& transform, float speed) {
	RW_LOG_DEBUG("rwhw::URCallBackInterface::moveT("<<transform<<","<<speed<<")");
    boost::mutex::scoped_lock lock(_mutex);

    popAllUpdateCommands();

    _commands.push(URScriptCommand(URScriptCommand::MOVET, transform));
    _robotStopped = false;
} 

void URCallBackInterface::servo(const rw::math::Q& q) {
	RW_LOG_DEBUG("rwhw::URCallBackInterface::servo("<<q<<")");
    boost::mutex::scoped_lock lock(_mutex);

    popAllUpdateCommands();
	
    _commands.push(URScriptCommand(URScriptCommand::SERVOQ, q, 1.0));
    _robotStopped = false;
	 
}


void URCallBackInterface::forceModeStart(const rw::math::Transform3D<>& base2ref, const rw::math::Q& selection, const rw::math::Wrench6D<>& wrench, const rw::math::Q& limits) {
    RW_LOG_DEBUG("rwhw::URCallBackInterface::forceModeStart("<<base2ref<<","<<selection<<","<<wrench<<","<<limits<<")");
	boost::mutex::scoped_lock lock(_mutex);

    popAllUpdateCommands();

    _commands.push(URScriptCommand(URScriptCommand::FORCE_MODE_START, base2ref, selection, wrench, limits));
    _robotStopped = false;
}

void URCallBackInterface::forceModeUpdate(const rw::math::Wrench6D<>& wrench) {
	RW_LOG_DEBUG("rwhw::URCallBackInterface::forceModeUpdate("<<wrench<<")");
    boost::mutex::scoped_lock lock(_mutex);

    popAllUpdateCommands();
    _commands.push(URScriptCommand(URScriptCommand::FORCE_MODE_UPDATE, wrench));

}

void URCallBackInterface::forceModeEnd() {
	RW_LOG_DEBUG("rwhw::URCallBackInterface::forceModeEnd()");
    boost::mutex::scoped_lock lock(_mutex);
    popAllUpdateCommands();
	URScriptCommand cmd(URScriptCommand::FORCE_MODE_END);
    _commands.push(cmd);
}

void URCallBackInterface::teachModeStart() {
	RW_LOG_DEBUG("rwhw::URCallBackInterface::teachModeStart()");
	if (_cb < CB3)
		RW_THROW("Teach Mode Start is not supported on controller boxes older than CB3");
	boost::mutex::scoped_lock lock(_mutex);
	popAllUpdateCommands();
	_commands.push(URScriptCommand(URScriptCommand::TEACH_MODE_START));
	_robotStopped = false;
}

void URCallBackInterface::teachModeEnd() {
	RW_LOG_DEBUG("rwhw::URCallBackInterface::teachModeEnd()");
	if (_cb < CB3)
		RW_THROW("Teach Mode End is not supported on controller boxes older than CB3");

	boost::mutex::scoped_lock lock(_mutex);
    popAllUpdateCommands();
    _commands.push(URScriptCommand(URScriptCommand::TEACH_MODE_END));
}

void URCallBackInterface::setDigitalOutput(int id, bool value) {
	RW_LOG_DEBUG("rwhw::URCallBackInterface::setDigitalOutput(id="<<id<<", value="<<value<<")");
	boost::mutex::scoped_lock lock(_mutex);
	_commands.push(URScriptCommand(URScriptCommand::SET_DIGOUT, id, value));
	_robotStopped = false;
}

void URCallBackInterface::setPayload(double mass, const Vector3D<>& centerOfGravity) {
	RW_LOG_DEBUG("rwhw::URCallBackInterface::setPayLoad("<<mass<<","<<centerOfGravity<<")");
    boost::mutex::scoped_lock lock(_mutex);
    popAllUpdateCommands();

    _commands.push(URScriptCommand(URScriptCommand::SET_PAYLOAD, mass, centerOfGravity));
    _robotStopped = false;
}
