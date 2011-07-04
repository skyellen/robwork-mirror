/* */
#include "UniversalRobotsRTLogging.hpp"
#include "URCommon.hpp"


using namespace rwhw;
using namespace rw::math;
using namespace rw::common;

UniversalRobotsRTLogging::UniversalRobotsRTLogging():
	_socket(NULL),
	_connected(false)
{

}

UniversalRobotsRTLogging::~UniversalRobotsRTLogging() {
	disconnect();
}

double UniversalRobotsRTLogging::driverTime() {
	return URCommon::driverTimeStamp();
}

void UniversalRobotsRTLogging::start() {
	_stop = false;
	_thread = ownedPtr(new boost::thread(boost::bind(&UniversalRobotsRTLogging::run, this)));
}

void UniversalRobotsRTLogging::stop() {
	_stop = true;
}

void UniversalRobotsRTLogging::run() {
	while (!_stop) {
		readRTInterfacePacket();
		//_thread->yield();
		boost::this_thread::sleep(boost::posix_time::milliseconds(2));
	}
}


void UniversalRobotsRTLogging::connect(const std::string& host, unsigned int port) {
	try {
		boost::asio::ip::tcp::resolver resolver(_ioService);
		boost::asio::ip::tcp::resolver::query query(host.c_str(), "");
		boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
		boost::asio::ip::tcp::endpoint ep = (*iter).endpoint();
		ep.port(port);
		_socket = new boost::asio::ip::tcp::socket(_ioService);
		_socket->connect(ep);
		_connected = true;
	} catch(boost::system::system_error& e) {
		RW_THROW("Unable to connect to command port with message: "<<e.what());
	}
}

void UniversalRobotsRTLogging::disconnect() {
	if(_socket != NULL) {
		_socket->shutdown(boost::asio::socket_base::shutdown_both);
		_socket->close();
		delete _socket;
		_connected = false;
	}
	_socket = NULL;
}


URRTData UniversalRobotsRTLogging::getLastData() {
	boost::mutex::scoped_lock lock(_mutex);
	return _data;
}

bool UniversalRobotsRTLogging::readRTInterfacePacket() {
	if(!_connected) {
		return false;
	}

    //Get the length of the available data
	size_t bytesReady = _socket->available();
	if (bytesReady < 4)
		return false;

	//If the first part of the packet is there, we will block until we have read the entire packet.

    unsigned int offset = 0;
    size_t msgSize = URCommon::getUInt32(_socket, offset);
    bytesReady = 0;
    int tries = 0;
    do {
		bytesReady = _socket->available();
	//	std::cout<<"-";
		tries += 1;

	} while (bytesReady < msgSize-4);

   // std::cout<<"Other message size = "<<msgSize<<std::endl;


    double timestamp = driverTime();
    double time = URCommon::getDouble(_socket, offset);
    //std::cout<<"Time = "<<time<<std::endl;

    Q q_target = URCommon::getQ(_socket, 6, offset);
    Q dq_target = URCommon::getQ(_socket, 6, offset);
    Q ddq_target = URCommon::getQ(_socket, 6, offset);

    Q i_target = URCommon::getQ(_socket, 6, offset);
    Q m_target = URCommon::getQ(_socket, 6, offset);

    Q q_actual = URCommon::getQ(_socket, 6, offset);
   // std::cout<<"q = "<<q_actual<<std::endl;
    Q dq_actual = URCommon::getQ(_socket, 6, offset);
    Q i_actual = URCommon::getQ(_socket, 6, offset);

    Q acc_values = URCommon::getQ(_socket, 18, offset);

    Q tcp_force = URCommon::getQ(_socket, 6, offset);
    Q tool_pose = URCommon::getQ(_socket, 6, offset);
    Q tcp_speed = URCommon::getQ(_socket, 6, offset);

    uint64_t digin = URCommon::getUInt64(_socket, offset);
   // std::cout<<"Digital Inputs"<<digin<<std::endl;
    //unsigned int digin1 = URCommon::getUInt32(_socket, offset);
    //unsigned int digin2 = URCommon::getUInt32(_socket, offset);
    //std::cout<<"Offset = "<<offset<<std::endl;
    //Q temperatures = getQ(socket, 6, offset);

    //If there is any data left, just read it to empty buffer
    char* buffer = new char[msgSize];
    _socket->read_some(boost::asio::buffer(buffer, msgSize-offset));

    boost::mutex::scoped_lock lock(_mutex);
    _data.driverTimeStamp = timestamp;
    _data.controllerTimeStamp = time;
    _data.qTarget = q_target;
    _data.dqTarget = dq_target;
    _data.ddqTarget = ddq_target;
    _data.iTarget = i_target;
    _data.torqueTarget = m_target;

    _data.qActual = q_actual;
    _data.dqActual = dq_actual;
    _data.iActual = i_actual;

    _data.accValues = acc_values;
    _data.tcpForce = tcp_force;
    _data.toolPose = tool_pose;
    _data.tcpSpeed = tcp_speed;
    _data.digIn = digin;
    return true;




}
