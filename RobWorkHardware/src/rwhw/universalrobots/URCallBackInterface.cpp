/* */
/* */
#include "URCallBackInterface.hpp"
#include "URCommon.hpp"
#include <rw/math/Transform3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/common/Timer.hpp>


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
}

bool URCallBackInterface::isMoving() const {
	return _isMoving;
}

double URCallBackInterface::driverTime() const {
	return URCommon::driverTimeStamp();
}

bool URCallBackInterface::connect(const std::string& host, unsigned int port) {
	return _urPrimary.connect(host, port);
}

void URCallBackInterface::startInterface(unsigned int callbackPort) {
	_callbackPort = callbackPort;
	_thread = ownedPtr(new boost::thread(boost::bind(&URCallBackInterface::run, this)));


	/*std::cout<<"Ready to load"<<std::endl;
	std::ifstream infile(filename.c_str());
	std::cout<<"Script Loaded"<<std::endl;
	// get length of file:
	infile.seekg (0, std::ios::end);
	long length = infile.tellg();
	infile.seekg (0, std::ios::beg);

	// allocate memory:
	char *buffer = new char [length+1];

	// read data as a block:
	infile.read (buffer,length);
    buffer[length] = 0;*/
    std::string script = UR_SCRIPT; //(buffer);
/*
      for (size_t i = 0; i<UR_SCRIPT.size(); i++) {
            if (UR_SCRIPT[i] == '\'')
                  UR_SCRIPT[i] = '\"';
      }
*/

    int n = script.find("PORT");
    if (n == std::string::npos)
        RW_THROW("Unable to find PORT in script");
    std::stringstream sstr;
    sstr<<script.substr(0, n)<<callbackPort<<script.substr(n+4);
	
    //std::cout<<"Send Script "<<sstr.str()<<std::endl;

      _urPrimary.sendScript(sstr.str());
}

void URCallBackInterface::startInterface(unsigned int callbackPort, const std::string& filename) {
	_callbackPort = callbackPort;
	_thread = ownedPtr(new boost::thread(boost::bind(&URCallBackInterface::run, this)));


	std::cout<<"Ready to load"<<std::endl;
	std::ifstream infile(filename.c_str());
	std::cout<<"Script Loaded"<<std::endl;
	// get length of file:
	infile.seekg (0, std::ios::end);
	long length = infile.tellg();
	infile.seekg (0, std::ios::beg);

	// allocate memory:
	char *buffer = new char [length+1];

	// read data as a block:
	infile.read (buffer,length);
      buffer[length] = 0;
      std::string script(buffer);

  /*    for (size_t i = 0; i<UR_SCRIPT.size(); i++) {
            if (UR_SCRIPT[i] == '\'')
                  UR_SCRIPT[i] = '\"';
      }


      if (script == UR_SCRIPT)
            std::cout<<"The scripts are the same"<<std::endl;
      else
            std::cout<<"There are differences"<<std::endl;



      std::cout<<"Sizes = "<<script.size()<<"  "<<UR_SCRIPT.size()<<std::endl;
      for (size_t i = 0; i<script.size(); i++) {
            std::cout<<"script["<<i<<"] = "<<(int)script.at(i)<<"  "<<(int)UR_SCRIPT.at(i)<<std::endl;
            if (script.at(i) != UR_SCRIPT.at(i))
                  std::cout<<"DIFFERENCE AT "<<i<<std::endl;
      }
*/
//      script = UR_SCRIPT;

      int n = script.find("PORT");
/*      if (n == std::string::npos)
        RW_THROW("Unable to find PORT in script");
*/
      std::stringstream sstr;
      sstr<<script.substr(0, n)<<callbackPort<<script.substr(n+4);

//      std::cout<<"Send Script "<<sstr.str()<<std::endl;

      _urPrimary.sendScript(sstr.str());
}



void URCallBackInterface::stopInterface() {
	_stopServer = true;
}


URPrimaryInterface& URCallBackInterface::getPrimaryInterface() {
	return _urPrimary;
}


namespace {

void q2intVector(const Q& q, std::vector<int>& integers, int offset) {
    for (size_t i = 0; i<q.size(); i++) {
        integers[i+1] = (int)(q(i)*10000);
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

}

void URCallBackInterface::sendStop(tcp::socket& socket) {
    std::vector<int> integers(8);
    integers[0] = URScriptCommand::STOP;
    URCommon::send(&socket, integers);    
}

void URCallBackInterface::handleCmdRequest(tcp::socket& socket) {
	boost::mutex::scoped_lock lock(_mutex);

//	std::cout<<"Handle Cmd Request ="<<_commands.size()<<std::endl;
    std::vector<int> integers(8);

	if (_commands.size() == 0 || (_isMoving && !_isServoing)) {
        integers[0] = URScriptCommand::DO_NOTHING;
        URCommon::send(&socket, integers);
     //   std::cout<<"Do Nothing"<<std::endl;
		return;
	}

	URScriptCommand cmd = _commands.front();

    _isServoing = false;
    integers[0] = cmd._type;
	switch (cmd._type) {
    case URScriptCommand::STOP:
        break;
	case URScriptCommand::MOVEQ:
		std::cout<<"Ready to execute move Q"<<std::endl;
        q2intVector(cmd._q, integers, 1);
        integers[7] = cmd._speed*10000;
		_isMoving = true;
		break;
	case URScriptCommand::MOVET: 
		std::cout<<"Ready to execute move Q"<<std::endl;
        t2intVector(cmd._transform, integers, 1);
        integers[7] = cmd._speed*10000;
		_isMoving = true;
		break;
	case URScriptCommand::SERVOQ: 
        q2intVector(cmd._q, integers, 1);
		_isMoving = true;
        _isServoing = true;
		break;	
    default:
        RW_THROW("Unsupported command type: "<<cmd._type);
        break;
	}
	URCommon::send(&socket, integers);


    if (cmd._type != URScriptCommand::SERVOQ) {                
      _commands.pop();
        std::cout<<"Pops commands"<<std::endl;
    }
}

void URCallBackInterface::run() {
  try
  {
	boost::asio::io_service io_service;

	tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), _callbackPort));
	while(!_stopServer)
	{
	  tcp::socket socket(io_service);
	  std::cout<<"Ready to accept incoming connections "<<std::endl;
	  acceptor.accept(socket);
	  std::cout<<"Incoming accepted"<<std::endl;
      Timer timer;
      timer.resetAndResume();  
	  while (!_stopServer) {
		//  std::cout<<"\b\b\b\b\bm = "<<_isMoving;
//          std::cout<<"Time = "<<TimerUtil::currentTimeUs()<<std::endl;
		  boost::system::error_code error;
		  size_t available = socket.available(error);
		  if (error == boost::asio::error::eof) {
			  std::cout<<"Reached EOF"<<std::endl;
			  break;
		  }
		  if (available >= 1) {
              char ch;
			  if (!URCommon::getChar(&socket, &ch))
                continue;
                
            if (_robotStopped) { 
                sendStop(socket);
            } else {
              if (ch == 0) {
                _isMoving = false;
              } 
              //std::cout<<"Time = "<<timer.getTime()<<std::endl;        
              timer.resetAndResume();
              handleCmdRequest(socket);
            }
//else if (ch == 1) {
                //Only handle something if we are using servoing
  //              handleServoUpdate(socket);                  
    //          }
/*			  if (str == "GET") {
				  std::string var = StringUtil::removeWhiteSpace(URCommon::readUntil(&socket, '\n', offset));
				//  std::cout<<"Data Recieved = "<<var<<std::endl;
				  if (var == "STOP") {
					  if (_robotStopped) {
						  URCommon::send(&socket, "STOP 1\n");
					  }
					  else {
						  URCommon::send(&socket, "STOP 0\n");
					  }

				  } else if (var == "CMD") {
					  handleCmdRequest(socket, var);
				  }
			  }
			  else if (str == "SET") {
				  std::string var = StringUtil::removeWhiteSpace(URCommon::readUntil(&socket, '\n', offset));
				  if (var.substr(0,3) == "FIN" && var.substr(3,1)=="1") {
					  _isMoving = false;
				  }
			  }*/
		  }
		  boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	  }
	}
  }
  catch (std::exception& e)
  {
	std::cerr << e.what() << std::endl;
  }



/*  try
  {
	boost::asio::io_service io_service;

	tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), _callbackPort));

    char buffer[256];

	while(!_stopServer)
	{
	  tcp::socket socket(io_service);
	  std::cout<<"Ready to accept incoming connections "<<std::endl;
	  acceptor.accept(socket);
	  std::cout<<"Incoming accepted"<<std::endl;
      Timer timer1;
      timer1.resetAndResume();      
	  while (!_stopServer) {
 		  boost::system::error_code error;
		  size_t available = socket.available(error);
		  if (error == boost::asio::error::eof) {
			  std::cout<<"Reached EOF"<<std::endl;
			  break;
		  }
         if (available >= 1) {
    		if (socket.read_some(boost::asio::buffer(buffer, 1))) {
              std::cout<<"Time = "<<timer1.getTime()<<" "<<(int)buffer[0]<<std::endl;
              timer1.reset();
              socket.send(boost::asio::buffer(buffer, 2));
//                socket.send(boost::asio::buffer(buffer, 1));
            }
        	

        }
	  }
	}
  }
  catch (std::exception& e)
  {
	std::cerr << e.what() << std::endl;
  }*/
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

void URCallBackInterface::popAllServoCommands() {
    while ((_commands.size() > 0) && (_commands.front()._type == URScriptCommand::SERVOQ)) {
        _commands.pop();
    }
}

void URCallBackInterface::moveQ(const rw::math::Q& q, float speed) {
	std::cout<<"Received a moveQ to "<<q<<std::endl;
    boost::mutex::scoped_lock lock(_mutex);
    
    popAllServoCommands();

    _commands.push(URScriptCommand(URScriptCommand::MOVEQ, q, speed));
    std::cout<<"Number of commands on queue = "<<_commands.size()<<std::endl;
    _robotStopped = false;
//    _isMoving = true;
}

void URCallBackInterface::moveT(const rw::math::Transform3D<>& transform, float speed) {
	std::cout<<"Received a moveT to "<<transform<<std::endl;
    boost::mutex::scoped_lock lock(_mutex);

    popAllServoCommands();

    _commands.push(URScriptCommand(URScriptCommand::MOVET, transform));
    _robotStopped = false;
//    _isMoving = true;
}

void URCallBackInterface::servo(const rw::math::Q& q) {
//	std::cout<<"Received a servoQ "<<q<<std::endl;
    boost::mutex::scoped_lock lock(_mutex);

    size_t n = _commands.size();
    popAllServoCommands();
    std::cout<<"Command Buffer Size "<<_commands.size()<<"  "<<n<<std::endl;

    _commands.push(URScriptCommand(URScriptCommand::SERVOQ, q, 1));
    _robotStopped = false;

}


