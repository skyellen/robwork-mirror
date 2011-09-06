#include "NetFTLogging.hpp"

// STL
#include <exception>

// RW
#include <rw/common/macros.hpp>

using namespace rwhw;
using namespace rw::common;
using namespace rw::math;

using namespace boost::asio;
using namespace boost::asio::ip;

NetFTLogging::NetFTLogging(const std::string& address,
             unsigned short port,
             unsigned int countsPerForce,
             unsigned int countsPerTorque) : _address(address),
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
    
void NetFTLogging::start() {
    // Open socket
    _socket.open(udp::v4());
    
    // Connect to endpoint
    udp::endpoint netftEndpoint(address_v4::from_string(_address), _port);
    _socket.connect(netftEndpoint);
    
    // Start receive thread
    _receiveThread = boost::thread(&NetFTLogging::runReceive, this);
    
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

void NetFTLogging::sendStartCommand() {
    // Command NetFT to start data transmission
    Command cmdStart;
    uint8_t cmdBuf[Command::RDT_COMMAND_SIZE];
    cmdStart.pack(cmdBuf);
    _socket.send(buffer(cmdBuf, Command::RDT_COMMAND_SIZE));
}

bool NetFTLogging::waitForNewData() {
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
NetFTLogging::~NetFTLogging() {
    stop();
}

void NetFTLogging::stop() {
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
void NetFTLogging::runReceive() {
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
                    Wrench3D tmpData;
                    tmpData.first[0] = double(msg._fx) * _scaleF;
                    tmpData.first[1] = double(msg._fy) * _scaleF;
                    tmpData.first[2] = double(msg._fz) * _scaleF;
                    tmpData.second[0] = double(msg._tx) * _scaleT;
                    tmpData.second[1] = double(msg._ty) * _scaleT;
                    tmpData.second[2] = double(msg._tz) * _scaleT;
                    
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

NetFTLogging::NetFTData NetFTLogging::getAllData() {
    // Instantiate return values
    unsigned int status, lost, count;
    Wrench3D data;
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
    
    return NetFTLogging::NetFTData(status, lost, count, data, timestamp);
}

Wrench3D NetFTLogging::getData() {
    // Instantiate return value
   Wrench3D data;
    
    // Acquire mutex
    {
        boost::unique_lock<boost::mutex> lock(_mutex);
        data = _data;
    }
    
    return data;
}

void NetFTLogging::print(std::ostream& os, const NetFTLogging::NetFTData& netftAllData) const {
    // Acquire data
    const Wrench3D& data = netftAllData.data;
    
    // Print
    os << "Status: " << netftAllData.status << std::endl;
    os << "Lost packets: " << netftAllData.lost << std::endl;
    os << "Packet count: " << netftAllData.count << std::endl;
    os << "Data {Fx, Fy, Fz, Tx, Ty, Tz}: {" << data.first[0] << ", " << data.first[1] << ", " << data.first[2] << ", "
                                             << data.second[0] << ", " << data.second[1] << ", " << data.second[2] << "}" << std::endl;
}

void NetFTLogging::print(std::ostream& os, const Wrench3D& netftData) const {
   os << netftData.first[0] << " " << netftData.first[1] << " " << netftData.first[2] << " "
      << netftData.second[0] << " " << netftData.second[1] << " " << netftData.second[2] << std::endl;
}
