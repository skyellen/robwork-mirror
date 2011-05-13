/* */
#ifndef RWHW_UNIVERSALROBOTSRTLOGGING_HPP
#define RWHW_UNIVERSALROBOTSRTLOGGING_HPP

#include <rw/math/Q.hpp>
#include <rw/common/types.hpp>
#include <boost/asio.hpp>

namespace rwhw {

class UniversalRobotsRTData {
public:
	UniversalRobotsRTData():
		digIn(0)
	{

	}

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
	rw::math::Q motorTemperatures;

};

class UniversalRobotsRTLogging {
	public:
		UniversalRobotsRTLogging();
		~UniversalRobotsRTLogging();

		void update();

		/**
		 * @brief Connects socket to UR on the real-time interface
		 *
		 * If not able to connect it throws a rw::common::Exception
		 * @param host [in] host address
		 * @param port [in] Port to connect to. Defaults to 30003.
		 */
		void connect(const std::string& host, unsigned int port = 30003);

		void disconnect();

    	bool readRTInterfacePacket();
	private:
		boost::asio::ip::tcp::socket* _socket;
		boost::asio::io_service _ioService;
		bool _connected;

};

} //end namespace

#endif //#ifndef RWHW_UNIVERSALROBOTSRTLOGGING_HPP
