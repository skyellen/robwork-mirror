/*
 * UniversalRobots.hpp
 *
 *  Created on: Apr 15, 2010
 *      Author: lpe
 */


#ifndef RWHW_UNIVERSALROBOTS_HPP
#define RWHW_UNIVERSALROBOTS_HPP
/**
 * @file UniversalRobots.hpp
 */

#include "UniversalRobotsData.hpp"

#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/task/Task.hpp>

//#include <sandbox/VelocityRamps/SyncVelocityRamp.hpp>
// Boost

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>



#include <fstream>

typedef boost::uint16_t uint16;
typedef boost::uint32_t uint32;

	namespace boost { namespace asio {
		namespace ip { 
			class tcp; 
		} 
	} }


namespace rwhw {

/**
 * @brief Implements the interface for a UR
 */
class UniversalRobots  {

public:
	typedef rw::common::Ptr<UniversalRobots> Ptr;

    /**
     * @brief Creates object
     */
    UniversalRobots();

    ~UniversalRobots();

	bool connectPrimary(const std::string& host, unsigned int port);
	bool connectRTInterface(const std::string& host, unsigned int port);

	bool sendScript(const std::string& filename);
	bool sendScriptAndOpenCallBack(const std::string& filename, unsigned int portControl);


	bool isConnectedPrimary() const;
	bool isConnectedControl() const;
	bool isConnectedRTInterface() const;

	void disconnect();
	void disconnectPrimary();
	void disconnectControl();
	void disconnectRTInterface();


	void update();
	bool moveTo(const rw::math::Q& q);
	bool executePath(const rw::trajectory::QPath& path);
	bool executeTrajectory(rwlibs::task::QTask::Ptr task, double speed, int id);


private:
	rw::models::Device::Ptr _device;
    rw::math::Q _sigma;
    rw::math::Q _tau;
    int _trajectoryId;


    //Socket
    boost::asio::ip::tcp::socket* connectSocket(const std::string &ip, unsigned int port, boost::asio::io_service& ioService);
    void disconnectSocket(boost::asio::ip::tcp::socket*& socket);

    bool readRTInterfacePacket(boost::asio::ip::tcp::socket* socket);

    void pathToScriptString(const rw::trajectory::QPath& path, std::ostringstream &stream);
    void qToScriptString(const rw::math::Q& q, int index, std::ostringstream &stream);
    bool servoJ(const rw::trajectory::QPath& path, const std::vector<double>& betas, double velScale, double accScale);
    bool moveJ(const rw::trajectory::QPath& path);


    bool readPacketPrimaryInterface(boost::asio::ip::tcp::socket* socket);
    void readRobotsState(boost::asio::ip::tcp::socket* socket, uint32& messageOffset, uint32& messageLength);

	unsigned char getUchar(boost::asio::ip::tcp::socket* socket, uint32 &messageOffset);
	uint16 getUINT16(boost::asio::ip::tcp::socket* socket, uint32 &messageOffset);
	uint32 getUINT32(boost::asio::ip::tcp::socket* socket, uint32 &messageOffset);
	float getFloat(boost::asio::ip::tcp::socket* socket, uint32 &messageOffset);
	double getDouble(boost::asio::ip::tcp::socket* socket, uint32 &messageOffset);
	long getLong(boost::asio::ip::tcp::socket* socket, uint32 &messageOffset);
	bool getBoolean(boost::asio::ip::tcp::socket* socket, uint32 &messageOffset);
	bool extractBoolean(uint16 input, unsigned int bitNumber);
	rw::math::Vector3D<double> getVector3D(boost::asio::ip::tcp::socket* socket, uint32 &messageOffset);
	rw::math::Q getQ(boost::asio::ip::tcp::socket* socket, int cnt, uint& messageOffset);

	bool getChar(boost::asio::ip::tcp::socket* socket, char* output);
	bool sendCommand(boost::asio::ip::tcp::socket* socket, const std::string &str);

	bool _haveReceivedSize;
	uint32 messageLength, messageOffset;

	boost::asio::ip::tcp::socket* _socketPrimary;
	boost::asio::io_service _ioServicePrimary;

	boost::asio::ip::tcp::socket* _socketControl;
	boost::asio::io_service _ioServiceControl;

	boost::asio::ip::tcp::socket* _socketRTInterface;
	boost::asio::io_service _ioServiceRTInterface;



	std::string _hostName;
//	unsigned int _cmdHostPort;
//	unsigned int _statusHostPort;

	bool _connectedPrimary;
	bool _connectedControl;
	bool _connectedRTInterface;
	static const unsigned int max_buf_len = 5000000;
	char buf[max_buf_len];

	//Data
	UniversalRobotsData _data;
	UniversalRobotsRTData _rtdata;

	bool _lastTimeRunningProgram;

	static const unsigned char ROBOT_STATE = 16, ROBOT_MESSAGE = 20, HMC_MESSAGE = 22;
	static const unsigned char ROBOT_MODE_DATA = 0, JOINT_DATA = 1, TOOL_DATA = 2, MASTERBOARD_DATA = 3, CARTESIAN_INFO = 4, LASER_POINTER_POSITION = 5;
	static const unsigned char _commandNumberPrecision = 14;
};

} //end namespace rwhw


#endif // end include guard
