#include "URCallBackInterface.hpp"
#include "URCommon.hpp"
#include <rw/math/Transform3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/common/Timer.hpp>
#include <rw/common/Log.hpp>

#include "urscript.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwhw;
using namespace boost::asio::ip;

URCallBackInterface::URCallBackInterface():
    _stopServer(false),
    _robotStopped(true),
    _isMoving(false)
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

bool URCallBackInterface::connect(const std::string& host, const unsigned int port) {
    return _urPrimary.connect(host, port);
}

/* Deprecated function */
void URCallBackInterface::startInterface(const unsigned int callbackPort, const std::string& filename) {
    startCommunication("192.168.100.1", callbackPort, filename);
}

void URCallBackInterface::startCommunication(const std::string& callbackIP, const unsigned int callbackPort, const std::string& filename) {
    _callbackPort = callbackPort;
    /* There should be some better error checking at the conversion of the IP from string to ip::address - but the documentation is limited with what error codes that can be returned / exception(s) that can be thrown - [look at the source for more specific error documentation/information] */
    _callbackIP = boost::asio::ip::address::from_string(callbackIP);

    // launch communication thread
    RW_LOG_DEBUG("Spawning communication thread");
    _thread = ownedPtr(new boost::thread(boost::bind(&URCallBackInterface::run, this)));

    // load UR script
    std::string script;
    if (!filename.empty()) { // load UR script from file
        RW_LOG_DEBUG("Loading UR script from the file: " << filename);
        std::cout << "Loading UR script from file " << filename << "..." << std::endl;
        std::ifstream infile(filename.c_str());
        std::cout << "Script loaded." << std::endl;
        // get length of file:
        infile.seekg (0, std::ios::end);
        long length = infile.tellg();
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
    int n = script.find("PORT");
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


/* Deprecated function */
void URCallBackInterface::stopInterface() {
    stopCommunication();
}

void URCallBackInterface::stopCommunication() {
    _stopServer = true;
}


URPrimaryInterface& URCallBackInterface::getPrimaryInterface() {
    return _urPrimary;
}


namespace {

    void q2intVector(const Q& q, std::vector<int>& integers, int offset) {
        for (size_t i = 0; i<q.size(); i++) {
            integers[offset+i] = (int)(q(i)*10000);
        }
    }

    void t2intVector(const Transform3D<>& transform, std::vector<int>& integers, int offset) {
	const Vector3D<>& p = transform.P();
	EAA<> eaa(transform.R());
    
        integers[offset] = p(0);
        integers[offset+1] = p(1);
        integers[offset+2] = p(2);
        integers[offset+3] = eaa(0);
        integers[offset+4] = eaa(1);
        integers[offset+5] = eaa(2);

    }

    void wrench2intVector(const Wrench6D<>& wrench, std::vector<int>& integers, int offset) {
	integers[offset] = wrench.force()(0)*10000;
	integers[offset+1] = wrench.force()(1)*10000;
	integers[offset+2] = wrench.force()(2)*10000;
	integers[offset+3] = wrench.torque()(0)*10000;
	integers[offset+4] = wrench.torque()(1)*10000;
	integers[offset+5] = wrench.torque()(2)*10000;
    }

}


void URCallBackInterface::sendStop(tcp::socket& socket) {
    std::vector<int> integers(8);
    integers[0] = URScriptCommand::STOP;
    URCommon::send(&socket, integers);    
}

void URCallBackInterface::handleCmdRequest(tcp::socket& socket) {
    RW_LOG_DEBUG("Handling command request - obtaining lock...");
    boost::mutex::scoped_lock lock(_mutex);
    RW_LOG_DEBUG("Obtained lock");

    //std::cout<<"Handle Cmd Request ="<<_commands.size()<<std::endl;
    std::vector<int> integers(8);

    if (_commands.size() == 0 || (_isMoving && !_isServoing)) {
        integers[0] = URScriptCommand::DO_NOTHING;
        URCommon::send(&socket, integers);
        RW_LOG_DEBUG("Do nothing");
        return;
    }

    URScriptCommand cmd = _commands.front();
    RW_LOG_DEBUG("Command type: " << cmd._type);

    _isServoing = false;
    integers[0] = cmd._type;
    switch (cmd._type) {
    case URScriptCommand::STOP:
        break;
    case URScriptCommand::MOVEQ:
        q2intVector(cmd._q, integers, 1);
        integers[7] = cmd._speed*10000;
        _isMoving = true;
        break;
    case URScriptCommand::MOVET: 
        t2intVector(cmd._transform, integers, 1);
        integers[7] = cmd._speed*10000;
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
        // Why nothing?
        break;
    default:
        RW_LOG_DEBUG("Unsupported command: " << cmd._type);
        RW_THROW("Unsupported command type: " << cmd._type);
        break;
    }
    RW_LOG_DEBUG("Sending command to robot...");
    URCommon::send(&socket, integers);
    RW_LOG_DEBUG("Command to robot sent");
    //std::cout<<"Sends: "<<std::endl;
    //BOOST_FOREACH(int i, integers) {
    //	std::cout<<i<<" "<<std::endl;
    //}

    if (cmd._type != URScriptCommand::SERVOQ) {                
        RW_LOG_DEBUG("Popping commands as the command type was not SERVOQ");
        _commands.pop();
    }
}

void URCallBackInterface::run() {
    try
    {
	boost::asio::io_service io_service;

	tcp::acceptor acceptor(io_service, tcp::endpoint(_callbackIP, _callbackPort));
	while(!_stopServer)
	{
            tcp::socket socket(io_service);
            std::cout<<"Ready to accept incoming connections "<<std::endl;
            RW_LOG_DEBUG("Ready to accept incoming connections on port '" << _callbackPort << "'");
            acceptor.accept(socket);
            RW_LOG_DEBUG("Incoming connection accepted");
            std::cout<<"Incoming accepted"<<std::endl;
            Timer timer;
            timer.resetAndResume();  
            while (!_stopServer) {
		//  std::cout<<"\b\b\b\b\bm = "<<_isMoving;
//          std::cout<<"Time = "<<TimerUtil::currentTimeUs()<<std::endl;
                boost::system::error_code error;
                size_t available = socket.available(error);
                if (available == 0) {
                    if (error == boost::asio::error::eof) {
                        std::cout<<"Reached EOF"<<std::endl;
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
                
                    RW_LOG_DEBUG("Received the byte/char '" << ch << "' from the connection.");

                    if (_robotStopped) {
                        RW_LOG_DEBUG("Robot is stopped, going to send stop.");
                        sendStop(socket);
                    } else {
                        if (ch == 0) {
                            _isMoving = false;
                        } 
                        RW_LOG_DEBUG("Going to handle command request - Time: " << timer.getTime());
                        timer.resetAndResume();
                        handleCmdRequest(socket);
                    }
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
    std::cout<<"RWHW Stop UR"<<std::endl;
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

void URCallBackInterface::moveQ(const rw::math::Q& q, float speed) {
    //std::cout<<"Received a moveQ to "<<q<<std::endl;
    boost::mutex::scoped_lock lock(_mutex);
    
    popAllUpdateCommands();

    _commands.push(URScriptCommand(URScriptCommand::MOVEQ, q, speed));
/* FIXME:
 * These kinds of output messages should be converted to RobWork debug (and/or log) messages, so they can easily be enabled and disabled.
 */
    std::cout<<"Number of commands on queue = "<<_commands.size()<<std::endl;
    _robotStopped = false;
//    _isMoving = true;
}

void URCallBackInterface::moveT(const rw::math::Transform3D<>& transform, float speed) {
    //std::cout<<"Received a moveT to "<<transform<<std::endl;
    boost::mutex::scoped_lock lock(_mutex);

    popAllUpdateCommands();

    _commands.push(URScriptCommand(URScriptCommand::MOVET, transform));
    _robotStopped = false;
//    _isMoving = true;
}

void URCallBackInterface::servo(const rw::math::Q& q) {
//	std::cout<<"Received a servoQ "<<q<<std::endl;
    boost::mutex::scoped_lock lock(_mutex);

    //size_t n = _commands.size();
    popAllUpdateCommands();
    //std::cout<<"Command Buffer Size "<<_commands.size()<<"  "<<n<<std::endl;

    _commands.push(URScriptCommand(URScriptCommand::SERVOQ, q, 1));
    _robotStopped = false;

}


void URCallBackInterface::forceModeStart(const rw::math::Transform3D<>& base2ref, const rw::math::Q& selection, const rw::math::Wrench6D<>& wrench, const rw::math::Q& limits) {
    boost::mutex::scoped_lock lock(_mutex);

    //size_t n = _commands.size();
    popAllUpdateCommands();
    //std::cout<<"Command Buffer Size "<<_commands.size()<<"  "<<n<<std::endl;

    _commands.push(URScriptCommand(URScriptCommand::FORCE_MODE_START, base2ref, selection, wrench, limits));
    _robotStopped = false;
}

void URCallBackInterface::forceModeUpdate(const rw::math::Wrench6D<>& wrench) {
    boost::mutex::scoped_lock lock(_mutex);

    size_t n = _commands.size();
    popAllUpdateCommands();
    std::cout<<"Command Buffer Size "<<_commands.size()<<"  "<<n<<std::endl;

    _commands.push(URScriptCommand(URScriptCommand::FORCE_MODE_UPDATE, wrench));

}


void URCallBackInterface::forceModeEnd() {
    boost::mutex::scoped_lock lock(_mutex);

    size_t n = _commands.size();
    popAllUpdateCommands();
    std::cout<<"Command Buffer Size "<<_commands.size()<<"  "<<n<<std::endl;

    _commands.push(URScriptCommand(URScriptCommand::FORCE_MODE_END));

}
