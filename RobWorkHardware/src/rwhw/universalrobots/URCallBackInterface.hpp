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

	void moveQ(const rw::math::Q& q);

	void moveT(const rw::math::Transform3D<>& transform);

private:
	URPrimaryInterface _urPrimary;

	rw::common::Ptr<boost::thread> _thread;


	unsigned int _callbackPort;

	bool _stopServer;
	bool _robotStopped;

	class URScriptCommand {
	public:

		enum CmdType { MOVEQ = 1, MOVET };

		URScriptCommand(CmdType type, const rw::math::Q& q):
			_type(MOVEQ),
			_q(q)
		{
		}

		URScriptCommand(CmdType type, const rw::math::Transform3D<>& transform):
			_type(MOVET),
			_transform(transform)
		{
		}


		CmdType _type;
		rw::math::Q _q;
		rw::math::Transform3D<> _transform;
	};

	std::queue<URScriptCommand> _commands;

	boost::mutex _mutex;

	void handleCmdRequest(boost::asio::ip::tcp::socket& socket, const std::string& name);

};

} //end namespace rwhw

#endif //#ifndef RWHW_URCALLBACKINTERFACE_HPP
