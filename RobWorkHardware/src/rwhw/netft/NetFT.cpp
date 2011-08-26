#include "NetFT.hpp"

// STL
#include <exception>

// RobWork
#include <rw/common/macros.hpp>
#include <rw/common/TimerUtil.hpp>

using namespace rwhw;
using namespace rw::common;

NetFT::NetFT(const std::string& address,
             unsigned short port,
             unsigned int countsPerForce,
             unsigned int countsPerTorque) : _data(6, 0.0),
                                             _address(address),
                                             _port(port),
                                             _socket(_ioservice),
                                             _countsF(countsPerForce),
                                             _countsT(countsPerTorque),
                                             _threadRunning(false),
                                             _stopThread(false),
                                             _lastRdtSequence(0),
                                             _systemStatus(0),
                                             _outOfOrderCount(0),
                                             _lostPackets(0),
                                             _packetCount(0) {
    // Set scaling
    _scaleF = 1.0 / _countsF;
    _scaleT = 1.0 / _countsT;
}
    
void NetFT::start() {
    // Open socket
    _socket.open(udp::v4());
    
    // Connect to endpoint
    udp::endpoint netftEndpoint(address_v4::from_string(_address), _port);
    _socket.connect(netftEndpoint);
    
    // Start receive thread
    _receiveThread = boost::thread(&NetFT::runReceive, this);
    
    // Command NetFT to start data transmission - retry up to ten times
    const unsigned int trials = 10;
    unsigned int i = 1;
    bool newData = false;
    do {
        sendStartCommand();
        newData = waitForNewData();
    } while(i++ <= trials && !newData);
    
    boost::unique_lock<boost::mutex> lock(_mutex);
    if(_packetCount == 0)
        RW_THROW("No data received from NetFT device");
}

void NetFT::sendStartCommand() {
    // Command NetFT to start data transmission
    Command cmdStart;
    uint8_t cmdBuf[Command::RDT_COMMAND_SIZE];
    cmdStart.pack(cmdBuf);
    _socket.send(buffer(cmdBuf, Command::RDT_COMMAND_SIZE));
}

bool NetFT::waitForNewData() {
    unsigned int currentPacketCount;
    {
        boost::unique_lock<boost::mutex> lock(_mutex);
        currentPacketCount = _packetCount;
    }

    // Wait up to 100ms for new packet
    const unsigned int trials = 10;
    unsigned int i = 1;
    bool arrived = false;
    do {
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        boost::unique_lock<boost::mutex> lock(_mutex);
        arrived = (_packetCount != currentPacketCount);
    } while(i++ <= trials && !arrived);
    

    return arrived;
}

// Destructor
NetFT::~NetFT() {
    stop();
}

void NetFT::stop() {
   if(_threadRunning) {
       _stopThread = true;
       // Give the thread one second to stop
       if(!_receiveThread.timed_join(boost::posix_time::seconds(1))) {
           // Failure, interrupt
           RW_WARN("Interrupting receive thread...");
           _receiveThread.interrupt();
           if(!_receiveThread.timed_join(boost::posix_time::seconds(1)))
               RW_WARN("Failed to interrupt receive thread");
       }
   }
   
   if(_socket.is_open()) {
      // Close socket
      _socket.shutdown(socket_base::shutdown_both);
      _socket.close();
   }
}

// Thread function
void NetFT::runReceive() {
    try {
        _threadRunning = true;
        Message msg;
        uint8_t buf[Message::RDT_RECORD_SIZE+1];
        while(!_stopThread) {
            // Receive
            const size_t len = _socket.receive(buffer(buf, Message::RDT_RECORD_SIZE+1));
            if(len == Message::RDT_RECORD_SIZE) {
            	_timestamp = TimerUtil::currentTime();
                // Parse data
                msg.unpack(buf);
                // Check status code and store if non-zero
                if(msg._status != 0) {
                    boost::unique_lock<boost::mutex> lock(_mutex);
                    _systemStatus = msg._status;
                }
                // Check sequence number
                const int32_t seqDiff = int32_t(msg._rdtSequence - _lastRdtSequence);
                _lastRdtSequence = msg._rdtSequence;
                if(seqDiff >= 1) {
                    // Store
                    std::vector<double> tmpData(6, 0.0);
                    tmpData[0] = double(msg._fx) * _scaleF;
                    tmpData[1] = double(msg._fy) * _scaleF;
                    tmpData[2] = double(msg._fz) * _scaleF;
                    tmpData[3] = double(msg._tx) * _scaleT;
                    tmpData[4] = double(msg._ty) * _scaleT;
                    tmpData[5] = double(msg._tz) * _scaleT;
                    
                    // Acquire mutex
                    boost::unique_lock<boost::mutex> lock(_mutex);
                    _data = tmpData;
                    _lostPackets += (seqDiff - 1);
                    ++_packetCount;
                } else {
                    // Acquire mutex
                    boost::unique_lock<boost::mutex> lock(_mutex);
                    // Don't use data that is old
                    ++_outOfOrderCount;
                }
            } else {
                // TODO: Packet size inconsistency
                RW_WARN("Packet size: " << len << " does not match expected size: " << Message::RDT_RECORD_SIZE);
            }
        }
    } catch(const std::exception& e) {
        _threadRunning = false;
        RW_WARN("Exception caught during reception: " << e.what());
    }
}

NetFT::NetFTData NetFT::getAllData() {
    // Instantiate return values
    unsigned int status, lost, count;
    std::vector<double> data;
    double timestamp;
    
    // Acquire mutex
    {
        boost::unique_lock<boost::mutex> lock(_mutex);
        status = _systemStatus;
        lost = _lostPackets;
        count = _packetCount;
        data = _data;
        timestamp = _timestamp;
    }
    
    return NetFT::NetFTData(status, lost, count, data, timestamp);
}

std::vector<double> NetFT::getData() {
    // Instantiate return value
    std::vector<double> data;
    
    // Acquire mutex
    {
        boost::unique_lock<boost::mutex> lock(_mutex);
        data = _data;
    }
    
    return data;
}


double NetFT::getDriverTime() {
	return TimerUtil::currentTime();
}

void NetFT::print(std::ostream& os, const NetFT::NetFTData& netftAllData) {
    // Acquire data
    const unsigned int &status = netftAllData.status,
                       &lost = netftAllData.lost,
                       &count = netftAllData.count;
    const std::vector<double>& data = netftAllData.data;
    
    // Print
    os << "Status: " << status << std::endl;
    os << "Lost packets: " << lost << std::endl;
    os << "Packet count: " << count << std::endl;
    os << "Data {Fx, Fy, Fz, Tx, Ty, Tz}: {" << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", " << data[4] << ", " << data[5] << "}" << std::endl;
}

void NetFT::print(std::ostream& os, const std::vector<double>& netftData) {
    for(std::vector<double>::const_iterator it = netftData.begin(); it != netftData.end(); ++it)
        os << *it << " ";
    os << std::endl;
}
