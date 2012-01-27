/* */
/* */
#include "URCallBackInterface.hpp"
#include "URCommon.hpp"
#include <rw/math/Transform3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/common/StringUtil.hpp>


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

/*Q URCallBackInterface::getServoSpeed(const Transform3D<>& transform, const VelocityScrew6D<>& velocity) {
	_device->setQ(_qservo, _state);
    Frame* tcpFrame = _device->getEnd();
    Transform3D<> Tcurrent = _device->baseTframe(tcpFrame, _state);
    Transform3D<> Tdiff = inverse(Tcurrent)*transform;


    VelocityScrew6D<> vs(Tdiff);
    double gain = 0.1;
    VelocityScrew6D<> diff = gain*(Tcurrent.R()*vs);
    diff += velocity;

    double linvel = diff.linear().norm2();

    const double maxLinearVelocity = 0.5;
    if (linvel > maxLinearVelocity) {
        diff *= maxLinearVelocity/linvel;
    }

    const double maxAngularVelocity = 0.5;
    if (diff.angular().angle() > maxAngularVelocity) {
        diff *= maxAngularVelocity/diff.angular().angle();
    }

    Q dqtarget = _xqp->solve(_qservo, _dqservo, diff, std::list<XQPController::Constraint>());
    _dqservo = dqtarget;
    _qservo += _dt*_dqservo;

    return _dqservo;


}*/

void URCallBackInterface::handleCmdRequest(tcp::socket& socket, const std::string& name) {
	boost::mutex::scoped_lock lock(_mutex);
	//std::cout<<"Handle Cmd Request "<<_commands.size()<<std::endl;
	if (_commands.size() == 0 || (_isMoving && !_isServoing)) {
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

    _isServoing = false;
	switch (cmd._type) {
	case URScriptCommand::MOVEQ:
		std::cout<<"Ready to execute move Q"<<std::endl;
		URCommon::send(&socket, cmd._q, cmd._speed);
		_isMoving = true;
		break;
	case URScriptCommand::MOVET: {
		std::cout<<"Ready to execute move Q"<<std::endl;
		const Vector3D<>& p = cmd._transform.P();
		EAA<> eaa(cmd._transform.R());
		Q q(6, p(0), p(1), p(2), eaa(0), eaa(1), eaa(2));
		URCommon::send(&socket, q, cmd._speed);
		_isMoving = true;
		break;
	}
	case URScriptCommand::SERVO: {
		std::cout<<"Ready to do some servoing"<<std::endl;
		URCommon::send(&socket, cmd._q, cmd._speed);
		//Q dq = getServoSpeed(cmd._transform, cmd.velocity);
		//URCommon::send(&socket, dq, 0);
		_isMoving = true;
        _isServoing = true;
		break;
	}
	}
//	if (cmd._type != URScriptCommand::SERVO)
		_commands.pop();


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
	  while (!_stopServer) {
		  std::cout<<"\b\b\bm="<<_isMoving;
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
	std::cout<<"RWHW Stop UR"<<std::endl;
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
//    _isMoving = true;
}

void URCallBackInterface::moveT(const rw::math::Transform3D<>& transform, float speed) {
	std::cout<<"Received a moveT to "<<transform<<std::endl;
    boost::mutex::scoped_lock lock(_mutex);
    _commands.push(URScriptCommand(URScriptCommand::MOVET, transform));
    _robotStopped = false;
//    _isMoving = true;
}

void URCallBackInterface::servo(const rw::math::Q& q) {
	std::cout<<"Received a servoQ "<<q<<std::endl;
    boost::mutex::scoped_lock lock(_mutex);
   // while (!_commands.empty())
   // 	_commands.pop();
    _commands.push(URScriptCommand(URScriptCommand::SERVO, q, 1));
    _robotStopped = false;

}

/*
void URCallBackInterface::servo(const Transform3D<>& transform) {
	std::cout<<"Received servoT "<<transform<<std::endl;

    boost::mutex::scoped_lock lock(_mutex);
    while (!_commands.empty())
    	_commands.pop();
    _commands.push(URScriptCommand(URScriptCommand::MOVET, transform));
    _robotStopped = false;


}*/
