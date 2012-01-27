/* */
/* */
#ifndef RWHW_URCALLBACKINTERFACE_HPP
#define RWHW_URCALLBACKINTERFACE_HPP

#include "URPrimaryInterface.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/Ptr.hpp>

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <queue>

namespace rwhw {



class URCallBackInterface {
public:
	URCallBackInterface();

	bool connect(const std::string& host, unsigned int port);

	void startInterface(const std::string& script, unsigned int callbackPort);

	void stopInterface();

	void run();

	URPrimaryInterface& getPrimaryInterface();

	void stopRobot();

	void moveQ(const rw::math::Q& q, float speed);

	void moveT(const rw::math::Transform3D<>& transform, float speed);

	void servo(const rw::math::Q& dq);

//	void servo(const rw::math::Transform3D<>& target);

	double driverTime() const;

	bool isMoving() const;

private:
	URPrimaryInterface _urPrimary;

	rw::common::Ptr<boost::thread> _thread;


	unsigned int _callbackPort;

	bool _stopServer;
	bool _robotStopped;

	bool _isMoving;
    bool _isServoing;

	class URScriptCommand {
	public:

		enum CmdType { MOVEQ = 1, MOVET, SERVO };

		URScriptCommand(CmdType type, const rw::math::Q& q, float speed):
			_type(type),
			_q(q),
			_speed(speed)
		{
		}

		URScriptCommand(CmdType type, const rw::math::Transform3D<>& transform):
			_type(type),
			_transform(transform)
		{
		}

		URScriptCommand(CmdType type, const rw::math::Transform3D<>& transform, const rw::math::VelocityScrew6D<>& velocity):
			_type(type),
			_transform(transform),
			_velocity(velocity)
		{}

		CmdType _type;
		rw::math::Q _q;
		rw::math::Transform3D<> _transform;
		rw::math::VelocityScrew6D<> _velocity;
		float _speed;
	};

	std::queue<URScriptCommand> _commands;

	boost::mutex _mutex;

	void handleCmdRequest(boost::asio::ip::tcp::socket& socket, const std::string& name);

	/** Stuff needed for the servoing */
	rw::math::Q _qcurrent;
	/*rw::math::Q _qservo;
	rw::math::Q _dqservo;
	double _dt;*/
//	rwlibs::algorithms::XQPController::Ptr _xqp;

};

} //end namespace rwhw

#endif //#ifndef RWHW_URCALLBACKINTERFACE_HPP
