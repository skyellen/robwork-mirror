#include "Robotiq.hpp"

#include <rw/common/TimerUtil.hpp>

#include <boost/thread.hpp>

#include <iostream>
#include <string>
#include <float.h>

#include <boost/detail/endian.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;

using namespace rwhw;
using namespace boost::asio::ip;

Robotiq::Robotiq(rw::math::Q currentQ,
                 rw::math::Q currentCurrent,
                 rw::math::Q target,
                 rw::math::Q speed,
                 rw::math::Q force,
                 unsigned int numberOfJoints):
        _stop(true),
        _haveReceivedSize(false),
        _socket(0),
        _connected(false),
        _packageIDCounter(0),
        _currentQ(currentQ),
        _currentCurrent(currentCurrent),
        _target(target),
        _speed(speed),
        _force(force),
        _numberOfJoints(numberOfJoints){
    assert(numberOfJoints == _currentQ.size());
    assert(numberOfJoints == _currentCurrent.size());
    assert(numberOfJoints == _target.size());
    assert(numberOfJoints == _speed.size());
    assert(numberOfJoints == _force.size());
}

Robotiq::~Robotiq() {
    disconnect();
}

bool Robotiq::connect(const std::string& ip, unsigned int port) {
    if (_connected) {
        RW_THROW("Already connected. Disconnect before connecting again!");
    }

    try {
        boost::asio::ip::tcp::resolver resolver(_ioService);
        boost::asio::ip::tcp::resolver::query query(ip.c_str(), "");
        boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
        boost::asio::ip::tcp::endpoint ep = (*iter).endpoint();
        ep.port(port);
        //Connecting to server
        _socket = new boost::asio::ip::tcp::socket(_ioService);
        _socket->connect(ep);

    } catch (boost::system::system_error& e) {
        RW_THROW("Unable to connect to command port with message: "<<e.what());
    }

    if (_socket == NULL) {
        _connected = false;
        RW_LOG_DEBUG("Could not connected to Robotiq hand on " << ip << " port " << port);
        return false;
    }
    _connected = true;

    RW_LOG_DEBUG("Connected to Robotiq hand on " << ip << " port " << port);

    // first thing we do is starting the thread
    start();
    // if gripper is not activated then activate it
    if (!isActivated()) {
        return activate(30);
    }

    return true;
}

void Robotiq::disconnect() {
    stop();
    _thread->join();
    _socket->close();
}

ModbusPackage Robotiq::send(ModbusPackage package) {
    if (!_connected) {
        RW_THROW("Unable to send command before connecting.");
    }

    setReg(package.header.data.transactionID, _packageIDCounter++);

    // reset the notification map
    _packagesIntransit[package.header.data.transactionID].second = false;

    _packagesOutgoing.push(package);
    // now wait for answer
    while (_packagesIntransit[package.header.data.transactionID].second == false) {
        // todo: check timeout
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }

    _packagesIntransit[package.header.data.transactionID].second = false;

    return _packagesIntransit[package.header.data.transactionID].first;
}

bool Robotiq::activate( unsigned int timeout) {

    RW_LOG_DEBUG("Activating hand");

    ModbusPackage package = getActivateRequestPackage();

    // this blocks until package answered or timeout
    ModbusPackage answer = send(package);

    // Checking response
    boost::uint16_t n;
    getReg(answer.header.data.length, n);
    if (n != 6 ||
        answer.data[0] != 0x00 ||
        answer.data[1] != 0x00 ||
        answer.data[2] != 0x00 ||
        answer.data[3] != 0x03) {
        RW_THROW("Received message is wrong size (is " << n << " should be " << 6 << ") or wrong content");
    }

    double startTime = TimerUtil::currentTime();
    TimerUtil::sleepMs(20);
    getAllStatusCMD();
    // get status until activation is complete
    while (isGripperInActivationProcess()) {
        if ((timeout != 0) && ((TimerUtil::currentTime() - startTime) > timeout)) {
            RW_LOG_DEBUG("Hand activation process timed out!");
            return false;
        }
        if (isGripperInReset()) {
            TimerUtil::sleepMs(20);
            send(package);
        }
        TimerUtil::sleepMs(20);
        getAllStatusCMD();
    }

    if (handAfterActivationConnected() != true) {
        RW_LOG_DEBUG("Heuristics say that the hand is not connected!");
        return false;
    }

    RW_LOG_DEBUG("Hand activated");
    return true;
}

void Robotiq::run() {
    ModbusPackage package;
    while (!_stop) {
        if (_connected) {
            // read status package from gripper
            {
                //Get the length of the available data
                uint32_t bytesReady = _socket->available();

                // if bytes available then read modbus header
                if (bytesReady > 7) {

                    _socket->read_some(boost::asio::buffer(&package.header.value[0], 8));

                    // now check how much data remains
                    boost::uint16_t n;
                    getReg(package.header.data.length, n);

                    // because we are only communicating with gripper this should never go above 20
                    if (n > 20) {
                        RW_WARN("Something is wrong in driver. Too much data received: " << n);
                        RW_WARN("Resetting socket");
                        abort();
                    }

                    // read the rest of the package
                    _socket->read_some(boost::asio::buffer(&package.data[0], n - 2));

                    // signal that the package was received
                    _packagesIntransit[package.header.data.transactionID].first = package;
                    _packagesIntransit[package.header.data.transactionID].second = true;

//                    // todo Move this to trace level instead of debug once trace is implemented
//                    if (Log::log().isEnabled(Log::Debug)) {
//                        std::stringstream traceMessage;
//                        traceMessage << "Received Header: ";
//                        traceMessage.fill('0');
//                        for (int i = 0; i < 8; i++) {
//                            traceMessage << std::setw(2) << std::hex << (unsigned int) package.header.value[i] << " ";
//                        }
//                        traceMessage << ": ";
//
//                        for (int i = 0; i < n - 2; i++) {
//                            traceMessage << std::setw(2) << std::hex << (unsigned int) package.data[i] << " ";
//                        }
//                        RW_LOG_DEBUG(traceMessage.str());
//                    }
                }
            }

            // write any requests to
            while (_packagesOutgoing.size() != 0) {
                package = _packagesOutgoing.back();
                _packagesOutgoing.pop();

                // set stuff in header
                boost::uint16_t n;
                package.header.data.unitID = 2; // not used
                package.header.data.protocolID = 0; // not used
                getReg(package.header.data.length, n);

//                // todo Move this to trace level instead of debug once trace is implemented
//                if (Log::log().isEnabled(Log::Debug)) {
//                    std::stringstream traceMessage;
//                    traceMessage << "Sending Header: ";
//                    traceMessage.fill('0');
//                    for (int i = 0; i < 8; i++) {
//                        traceMessage << std::setw(2) << std::hex << (unsigned int) package.header.value[i] << " ";
//                    }
//                    traceMessage << ": ";
//                    for (int i = 0; i < n - 2; i++) {
//                        traceMessage << std::setw(2) << std::hex << (unsigned int) package.data[i] << " ";
//                    }
//                    RW_LOG_DEBUG(traceMessage.str());
//                }

                std::size_t bytesToBeTransfered = 8 + n - 2;
                std::size_t bytesTransfered = 0;
                bytesTransfered = boost::asio::write(
                        *_socket, boost::asio::buffer(&package.header.value[0], bytesToBeTransfered));
                if (bytesTransfered == bytesToBeTransfered) {
                    /* Successful sent */
                    RW_LOG_DEBUG("Sent all of the '" << bytesTransfered << "' bytes.");
                } else {
                    /* Unsuccessful sent */
                    RW_LOG_DEBUG("Unable to send all the '" << bytesToBeTransfered << "' bytes - only sent '" << bytesTransfered << "' bytes");
                }
            }

            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        } else {
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
        _thread->yield();
    }
}

void Robotiq::start() {
    _thread = ownedPtr(new boost::thread(&Robotiq::run, this));
    _stop = false;
}

void Robotiq::stop() {
    _stop = true;
}

void Robotiq::stopCmd() {
    ModbusPackage package = getStopCMDRequestPackage();

    ModbusPackage answer = send(package);

    validateStopCMDResponseMessage(answer);
}

void Robotiq::moveCmd(){
    // set target and move to it
    Q target = _target;

    ModbusPackage package = getMoveCMDRequestPackage(target);

    // this blocks until package answered or timeout
    ModbusPackage answer = send(package);

    // number of data bytes to use, should be 0
    if( answer.data[0] != 0 )
        RW_THROW("answer should be 0 not "<< answer.data[0]);

}

void Robotiq::moveCmd(rw::math::Q target) {
    std::pair<Q, Q> lim = getLimitPos();
    _target = Math::clampQ(target, lim.first, lim.second);
    moveCmd();
}

void Robotiq::getAllStatusCMD() {
    ModbusPackage package = getAllStatusCMDRequestPackage();

    // this blocks until package answered or timeout
    ModbusPackage answer = send(package);

    validateStatusPackageAndUpdateState(answer);
}

void Robotiq::setTargetQ(const rw::math::Q& target) {
    std::pair<Q, Q> lim = getLimitPos();
    _target = Math::clampQ(target, lim.first, lim.second);
}

void Robotiq::setTargetQVel(const rw::math::Q& jointVel) {
    std::pair<Q, Q> lim = getLimitVel();
    _speed = Math::clampQ(jointVel, lim.first, lim.second);
}

void Robotiq::setTargetQForce(const rw::math::Q& jointForce) {
    std::pair<Q, Q> lim = getLimitForce();
    _force = Math::clampQ(jointForce, lim.first, lim.second);
}

rw::math::Q Robotiq::getTargetQ() {
    return _target;
}

rw::math::Q Robotiq::getQ() {
    return _currentQ;
}

rw::math::Q Robotiq::getQCurrent() {
    return _currentCurrent;
}

double Robotiq::getCurrentInAmpereFromTicks(int ticks) const {
    // ticks are 0..255 (as coming back from hand)
    // return in ampere
    return 0.1 * ticks / 1000.0;
}

unsigned int Robotiq::getNumberOfJoints() const {
    return _numberOfJoints;
}

bool Robotiq::handAfterActivationConnected() const {
    for (size_t i=0; i < _currentCurrent.size(); ++i) {
        if ((_currentCurrent(i) != 0) || (_currentQ(i) != 0)) {
            return true;
        }
    }
    return false;
}

void Robotiq::setReg(boost::uint8_t& reg, const boost::uint8_t& val)  const{
    reg = val;
}

void Robotiq::setReg(boost::uint16_t& reg, const boost::uint16_t& val) const{
#if !defined(BOOST_BIG_ENDIAN)
    uint8_t val1 = val & 0xFF;
    uint8_t val2 = (val >> 8) & 0xFF;
    reg = val1 << 8 | val2;
#else
    reg = val;
#endif

}

void Robotiq::getReg(const boost::uint16_t& reg, boost::uint16_t& val) const{
#if !defined(BOOST_BIG_ENDIAN)
    uint8_t val1 = reg & 0xFF;
    uint8_t val2 = (reg >> 8) & 0xFF;
    val = val1 << 8 | val2;
#else
    val = reg;
#endif

}

boost::uint8_t Robotiq::toVal8(const int val) const{
    int tmp = val;
    if (tmp > 0xFF) return 0xFF;
    if (tmp < 0) return 0;
    return tmp & 0xFF;
}
