/* */
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
	URRTData():
		digIn(0)
	{

	}

	/**
	 * @brief Timestamp of when data arrived on the PC
	 */
	double driverTimeStamp;

	/**
	 * @bruef Timestamp given to data by the UR controller
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


	rw::math::Q tcpForce;
	rw::math::Q toolPose;
	rw::math::Q tcpSpeed;
	rw::math::Q toolTargetPose;
	rw::math::Q toolTargetSpeed;

	int64_t digIn;
	rw::math::Q motorTemperatures;

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

		void disconnect();

    	bool readRTInterfacePacket();

    	bool hasData() const;
    	URRTData getLastData();

    	double driverTime();
    	bool _lostConnection;
    	long long _lastPackageTime;
	private:
		boost::asio::ip::tcp::socket* _socket;
		boost::asio::io_service _ioService;
		rw::common::Ptr<boost::thread> _thread;
		boost::mutex _mutex;
		bool _connected;
		bool _stop;
		void run();

		bool _hasData;
		URRTData _data;

		std::string _host;
		unsigned int _port;
		bool _reestablishConnection;


};

} //end namespace

#endif //#ifndef RWHW_UNIVERSALROBOTSRTLOGGING_HPP
