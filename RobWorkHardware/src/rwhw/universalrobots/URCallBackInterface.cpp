/* */
/* */
#include "URCallBackInterface.hpp"
#include "URCommon.hpp"

#include <rw/common/StringUtil.hpp>


using namespace rw::common;
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

void URCallBackInterface::startInterface(const std::string& filename, unsigned int callbackPort) {
	_callbackPort = callbackPort;
	_thread = ownedPtr(new boost::thread(boost::bind(&URCallBackInterface::run, this)));
	_urPrimary.sendScript(filename);
}

void URCallBackInterface::stopInterface() {
	_stopServer = true;
}


URPrimaryInterface& URCallBackInterface::getPrimaryInterface() {
	return _urPrimary;
}


void URCallBackInterface::handleCmdRequest(tcp::socket& socket, const std::string& name) {
	boost::mutex::scoped_lock lock(_mutex);
	//std::cout<<"Handle Cmd Request "<<_commands.size()<<std::endl;
	if (_commands.size() == 0) {
		std::stringstream sstr;
		sstr<<name<<" "<<0<<"\n";
	//	std::cout<<"Send 0 Command = "<<sstr.str()<<std::endl;
		URCommon::send(&socket, sstr.str());
		return;
	}

	URScriptCommand cmd = _commands.front();

	std::stringstream sstr;
	sstr<<name<<" "<<cmd._type<<"\n";
	std::cout<<"Send Command = "<<sstr.str()<<std::endl;
	URCommon::send(&socket, sstr.str());

	switch (cmd._type) {
	case URScriptCommand::MOVEQ:
		std::cout<<"Ready to execute move Q"<<std::endl;

		URCommon::send(&socket, cmd._q, cmd._speed);

		break;
	case URScriptCommand::MOVET:
		//URCommon::send(&socket, cmd._transform);
		break;
	}

	_commands.pop();


}

void URCallBackInterface::run() {
  try
  {
	boost::asio::io_service io_service;

	tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), _callbackPort));
	int cnt = 0;
	while(!_stopServer)
	{
	  tcp::socket socket(io_service);
	  std::cout<<"Ready to accept incoming connections "<<std::endl;
	  acceptor.accept(socket);
	  std::cout<<"Incoming accepted"<<std::endl;
	  while (!_stopServer) {
		  boost::system::error_code error;
		  size_t available = socket.available(error);
		  if (error == boost::asio::error::eof) {
			  std::cout<<"Reached EOF"<<std::endl;
			  break;
		  }
		  //if (available != 0)
		  //	  std::cout<<cnt++<<"Available "<<available<<std::endl;
		  if (available > 3) {
			  unsigned int offset = 0;
			  std::string str = URCommon::getString(&socket, 3, offset);
			//  std::cout<<"str = "<<str<<std::endl;
			  if (str == "GET") {
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
				//  std::cout<<"Data Recieved = "<<var<<std::endl;
				  if (var.substr(0,3) == "FIN" && var.substr(3,1)=="1") {
					  _isMoving = false;
				  }
			  }
		  }
		  boost::this_thread::sleep(boost::posix_time::milliseconds(2));
		  //_thread->yield();
	  }
	}
  }
  catch (std::exception& e)
  {
	std::cerr << e.what() << std::endl;
  }
}


void URCallBackInterface::stopRobot() {
	//std::cout<<"RWHW Stop UR"<<std::endl;
	//_commands.push(URScriptCommand(URScriptCommand::STOP));
	_robotStopped = true;
	_isMoving = false;
}

void URCallBackInterface::moveQ(const rw::math::Q& q, float speed) {
	std::cout<<"Received a moveQ to "<<q<<std::endl;
    boost::mutex::scoped_lock lock(_mutex);
    _commands.push(URScriptCommand(URScriptCommand::MOVEQ, q, speed));
    std::cout<<"Number of commands on queue = "<<_commands.size()<<std::endl;
    _robotStopped = false;
    _isMoving = true;
}

void URCallBackInterface::moveT(const rw::math::Transform3D<>& transform) {
    boost::mutex::scoped_lock lock(_mutex);
    _commands.push(URScriptCommand(URScriptCommand::MOVET, transform));
    _robotStopped = false;
    _isMoving = true;
}
