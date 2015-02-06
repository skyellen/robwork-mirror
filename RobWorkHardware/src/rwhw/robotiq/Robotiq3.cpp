/*
 * UniversalRobots.cpp
 *
 *  Created on: Apr 15, 2010
 *      Author: lpe
 */

#include "Robotiq3.hpp"

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

static boost::uint16_t FC04 = 0x04;
static boost::uint16_t FC16 = 0x10;

namespace {

    void setReg(boost::uint8_t& reg, boost::uint8_t val){
        reg = val;
    }

    void setReg(boost::uint16_t& reg, boost::uint16_t val){
#if !defined(BOOST_BIG_ENDIAN)
        uint8_t val1 = val&0xFF;
        uint8_t val2 = (val>>8)&0xFF;
        reg = val1<<8 | val2;
#else
        reg = val;
#endif

    }

    void getReg(boost::uint16_t& reg, boost::uint16_t& val){
#if !defined(BOOST_BIG_ENDIAN)
        uint8_t val1 = reg&0xFF;
        uint8_t val2 = (reg>>8)&0xFF;
        val = val1<<8 | val2;
#else
       val = reg;
#endif

    }

}


Robotiq3::Robotiq3():
    _haveReceivedSize(false),
    _socket(0),
    _connected(false),
    _currentQ(4,0,0,0,0),
    _currentSpeed(4,0,0,0,0),
    _currentForce(4,0,0,0,0),
    _target(4,0,0,0,0),
    _speed(4,1,1,1,1),
    _force(4,1,1,1,1)
{


}

Robotiq3::~Robotiq3() {
    disconnect();
}

bool Robotiq3::connect(const std::string& ip, unsigned int port) {
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

    } catch(boost::system::system_error& e) {
        RW_THROW("Unable to connect to command port with message: "<<e.what());
    }

    if (_socket == NULL) {
        _connected = false;
        RW_LOG_DEBUG("Could not connected to Robotiq3 on " << ip << " port " << port);
        return false;
    }
    _connected = true;

    RW_LOG_DEBUG("Connected to Robotiq3 on " << ip << " port " << port);


    // first thing we do is starting the thread
    start();
    // if gripper is not activated then activate it
    if(!isActivated()){
        activate();
    }

    return true;
}

void Robotiq3::disconnect(){
    stop();
    _thread->join();
    _socket->close();
}

Robotiq3::ModbusPackage Robotiq3::send(ModbusPackage package){
    if(!_connected) {
        RW_THROW("Unable to send command before connecting.");
    }

    setReg(package.header.data.transactionID, _packageIDCounter++);

    // reset the notification map
    _packagesIntransit[package.header.data.transactionID].second = false;

    _packagesOutgoing.push(package);
    // now wait for answer
    while( _packagesIntransit[package.header.data.transactionID].second == false ){
        // todo: check timeout
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }

    _packagesIntransit[package.header.data.transactionID].second = false;

    return _packagesIntransit[package.header.data.transactionID].first;
}


void Robotiq3::activate(){
    ModbusPackage package;

    RW_LOG_DEBUG("Activating hand");

    setReg( package.header.data.functionCode, FC16);
    setReg( package.header.data.length, 13);

    // register start address
    //package.data[0] = 0x03 ;
    //package.data[1] = 0xE8;
    package.data[0] = 0x00;
    package.data[1] = 0x00;

    // number of registers
    package.data[2] = 0x00;
    package.data[3] = 0x03;
    // number data bytes to follow
    package.data[4] = 0x06;

    package.data[5] = 0x01; // action request
    package.data[6] = 0x00; // gripper options

    package.data[7] = 0x00; // register 0x0001
    package.data[8] = 0x00;

    package.data[9] = 0x00; // register 0x0002
    package.data[10] = 0x00;

    // this blocks until package answered or timeout
    ModbusPackage answer = send(package);

    // Checking response
    boost::uint16_t n;
    getReg(answer.header.data.length,n);
    if( n!=6 ||
        answer.data[0] != 0x00 ||
        answer.data[1] != 0x00 ||
        answer.data[2] != 0x00 ||
        answer.data[3] != 0x03) {
        RW_THROW("Received message is wrong size (is " << n << " should be " << 6 << ") or wrong content");
    }

    TimerUtil::sleepMs(20);
    getAllStatus();
    // get status until activation is complete
    while( _gripperStatus.data.gIMC==1 || _gripperStatus.data.gACT==0 ){
        if( _gripperStatus.data.gIMC==0 ) {
            TimerUtil::sleepMs(20);
            send(package);
        }
        TimerUtil::sleepMs(20);
        getAllStatus();
    }
    RW_LOG_DEBUG("Hand activated");
}

void Robotiq3::getAllStatus() {
    ModbusPackage package;

    setReg(package.header.data.functionCode, FC04);
    setReg(package.header.data.length, 6);
    setReg(package.header.data.unitID, 2);

    // register start address
    package.data[0] = 0x00;
    package.data[1] = 0x00;
    // number of registers
    package.data[2] = 0x00;
    package.data[3] = 0x08;

    // this blocks until package answered or timeout
    ModbusPackage answer = send(package);

    // number of data bytes to use, should be 16
    boost::uint16_t n;
    getReg(answer.header.data.length, n);
    if( answer.data[0] != 16 || n != 16+3)
        RW_THROW("answer should contain 16 bytes and not "<< (int) answer.data[0] << " and the length should be 19 instead of " << n);

    // now read everything into internal variables
    for(int i=0;i<16;i++){
        _status.value[i] = answer.data[1+i];
    }

    _gripperStatus.value = answer.data[1];
    _objectStatus.value = answer.data[2];
    _faultStatus.value = answer.data[3];

    std::pair<Q,Q> lim = getLimitPos();

    _currentQ(0) = lim.first(0) + (_status.data._posA / 255.0)*(lim.second(0)-lim.first(0));
    _currentQ(1) = lim.first(1) + (_status.data._posB / 255.0)*(lim.second(1)-lim.first(1));
    _currentQ(2) = lim.first(2) + (_status.data._posC / 255.0)*(lim.second(2)-lim.first(2));
    _currentQ(3) = lim.first(3) + (_status.data._posScissor / 255.0)*(lim.second(3)-lim.first(3));

    //_currentSpeed(0)
    _currentForce(0)  = _status.data._forceA * 1.0/255.0;
    _currentForce(1)  = _status.data._forceB * 1.0/255.0;
    _currentForce(2)  = _status.data._forceC * 1.0/255.0;
    _currentForce(3)  = _status.data._forceScissor * 1.0/255.0;

    _statusTimeStamp = TimerUtil::currentTimeMs();

    // todo Move this to trace level instead of debug once trace is implemented
    if (Log::log().isEnabled(Log::Debug)) {
        std::stringstream traceMessage;
        traceMessage << "Status: ";
        traceMessage.fill('0');
        for(int i=0;i<16;i++){
            traceMessage << std::setw(2) << std::hex << (unsigned int)_status.value[i] << " ";
        }
        traceMessage << std::endl;
        traceMessage << "gIMC:"<< _gripperStatus.data.gIMC << " gACT:"<<_gripperStatus.data.gACT;
        RW_LOG_DEBUG(traceMessage.str());
    }
}


void Robotiq3::run() {
    ModbusPackage package;
    while (!_stop) {
        if (_connected) {
            // read status package from gripper
            {
                //Get the length of the available data
                uint32_t bytesReady = _socket->available();

                // if bytes available then read modbus header
                if(bytesReady>7){

                    _socket->read_some(boost::asio::buffer(&package.header.value[0], 8));

                    // now check how much data remains
                    boost::uint16_t n;
                    getReg(package.header.data.length, n);

                    // because we are only communicating with gripper this should never go above 20
                    if(n>20){
                        RW_WARN("Something is wrong in driver. Too much data received: " << n);
                        RW_WARN("Resetting socket");
                        abort();
                    }

                    // read the rest of the package
                    _socket->read_some(boost::asio::buffer(&package.data[0], n-2));

                    // signal that the package was received
                    _packagesIntransit[package.header.data.transactionID].first = package;
                    _packagesIntransit[package.header.data.transactionID].second = true;

                    // todo Move this to trace level instead of debug once trace is implemented
                    if (Log::log().isEnabled(Log::Debug)) {
                        std::stringstream traceMessage;
                        traceMessage << "Received Header: ";
                        traceMessage.fill('0');
                        for(int i=0;i<8;i++){
                            traceMessage << std::setw(2) << std::hex << (unsigned int)package.header.value[i] << " ";
                        }
                        traceMessage << ": ";

                        for(int i=0;i<n-2;i++){
                            traceMessage << std::setw(2) << std::hex << (unsigned int)package.data[i] << " ";
                        }
                        RW_LOG_DEBUG(traceMessage.str());
                    }
                }
            }

            // write any requests to
            while(_packagesOutgoing.size()!=0){
                package = _packagesOutgoing.back();
                _packagesOutgoing.pop();

                // set stuff in header
                boost::uint16_t n;
                package.header.data.unitID = 2; // not used
                package.header.data.protocolID = 0; // not used
                getReg(package.header.data.length,n);

                // todo Move this to trace level instead of debug once trace is implemented
                if (Log::log().isEnabled(Log::Debug)) {
                    std::stringstream traceMessage;
                    traceMessage << "Sending Header: ";
                    traceMessage.fill('0');
                    for(int i=0;i<8;i++){
                        traceMessage << std::setw(2) << std::hex << (unsigned int)package.header.value[i] << " ";
                    }
                    traceMessage << ": ";
                    for(int i=0;i<n-2;i++){
                        traceMessage << std::setw(2) << std::hex << (unsigned int)package.data[i] << " ";
                    }
                    RW_LOG_DEBUG(traceMessage.str());
                }


                std::size_t bytesToBeTransfered = 8 + n-2;
                std::size_t bytesTransfered = 0;
                bytesTransfered = boost::asio::write(*_socket, boost::asio::buffer(&package.header.value[0], bytesToBeTransfered));
                if (bytesTransfered == bytesToBeTransfered) {
                    /* Successful send */
                    RW_LOG_DEBUG("Sent all of the '" << bytesTransfered << "' bytes.");
                } else {
                    /* Unsuccessful send */
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

void Robotiq3::start() {
    _thread = ownedPtr(new boost::thread(&Robotiq3::run, this));
    _stop = false;
}

void Robotiq3::stop() {
    _stop = true;
}

namespace {

    boost::uint8_t toVal8(int val){
        int tmp = val*0xFF;
        if(tmp>0xFF)
            return 0xFF;
        if(tmp<0)
            return 0;
        return tmp & 0xFF;
    }

}

void Robotiq3::stopCmd(){
    ModbusPackage package;

    setReg( package.header.data.functionCode, FC16);
    setReg( package.header.data.length, 9);

    // register start address
    package.data[0] = 0x00;
    package.data[1] = 0x00;

    // number of registers
    package.data[2] = 0x00;
    package.data[3] = 0x01;
    // number data bytes to follow
    package.data[4] = 0x02;

    package.data[5] = 0x01; // action request
    package.data[6] = 0x0c; // gripper options

    ModbusPackage answer = send(package);

    // check if response is correct
    boost::uint16_t n;
    getReg(answer.header.data.length,n);
    if( n!=6 ||
        answer.data[0] != 0x00 ||
        answer.data[1] != 0x00 ||
        answer.data[2] != 0x00 ||
        answer.data[3] != 0x01)
        RW_THROW("Received message is wrong size (is " << n << " should be " << 6 << ") or wrong content");
}

void Robotiq3::moveCmd(bool block){
    // move to target
    //
    // set target and move to it
    ModbusPackage package;

    Q target = _target;

    std::pair<Q,Q> lim = getLimitPos();

    setReg(package.header.data.functionCode, FC16);
    setReg(package.header.data.length, 2 + 5 + 16);

    ActionRequest actreq;
    actreq.data.rACT = 1;
    actreq.data.rMOD = 0; // basic mode
    actreq.data.rGTO = 1;

    GripperOptions gripopt;
    gripopt.value = 0x00;
    gripopt.data.rICF = 1; // individual finger control
    gripopt.data.rICS = 1; // individual control of scissor. disable mode selection

    ActionRequestCMD cmd;
    cmd.data._actionRequest = actreq.value;
    cmd.data._gripperOptions = gripopt.value;
    cmd.data._gripperOptions2 = 0x00;

    cmd.data._posA_req = ((target[0] - lim.first[0])/(lim.second[0] - lim.first[0]) * 0xFF) ;
    cmd.data._posB_req = ((target[1] - lim.first[1])/(lim.second[1] - lim.first[1]) * 0xFF) ;
    cmd.data._posC_req = ((target[2] - lim.first[2])/(lim.second[2] - lim.first[2]) * 0xFF) ;
    cmd.data._posScissor_req = ((target[3] - lim.first[3])/(lim.second[3] - lim.first[3]) * 0xFF) ;

    cmd.data._speedA = toVal8(_speed[0]);
    cmd.data._speedB = toVal8(_speed[1]);
    cmd.data._speedC = toVal8(_speed[2]);
    cmd.data._speedScissor = toVal8(_speed[3]);

    cmd.data._forceA = toVal8(_force[0]);
    cmd.data._forceB = toVal8(_force[1]);
    cmd.data._forceC = toVal8(_force[2]);
    cmd.data._forceScissor = toVal8(_force[3]);


    // register start address
    package.data[0] = 0x00;
    package.data[1] = 0x00;
    // number of registers
    package.data[2] = 0x00;
    package.data[3] = 0x08;
    // number data to follow
    package.data[4] = 16;

    // now come the positions, speed and forces
    for(int i=0;i<16;i++){
        package.data[5+i] = cmd.value[i];
    }

    // this blocks until package answered or timeout
    ModbusPackage answer = send(package);

    // number of data bytes to use, should be 0
    if( answer.data[0] != 0 )
        RW_THROW("answer should be 0 not "<< answer.data[0]);

}

void Robotiq3::moveCmd(rw::math::Q target, bool block){
    std::pair<Q,Q> lim = getLimitPos();
    _target = Math::clampQ(target, lim.first, lim.second );
    moveCmd(block);
}

void Robotiq3::moveJointCmd(int jointIdx, double target, bool block){ }

bool Robotiq3::waitCmd(double timeout){ return true; }

void Robotiq3::setTargetQ(const rw::math::Q& target){
    std::pair<Q,Q> lim = getLimitPos();
    _target = Math::clampQ(target, lim.first, lim.second );
}

void Robotiq3::setTargetQVel(const rw::math::Q& jointVel){
    std::pair<Q,Q> lim = getLimitVel();
    _speed = Math::clampQ(jointVel, lim.first, lim.second );
}

void Robotiq3::setTargetQAcc(const rw::math::Q& jointAcc){

}

void Robotiq3::setTargetQCurrent(const rw::math::Q& jointCurr){
    std::pair<Q,Q> lim = getLimitCurr();
    _force = Math::clampQ(jointCurr, lim.first, lim.second );
}

rw::math::Q Robotiq3::getTargetQ(){ return _target; }

rw::math::Q Robotiq3::getQ(){  return _currentQ; }

rw::math::Q Robotiq3::getdQ(){ return _currentSpeed; }

rw::math::Q Robotiq3::getQCurrent(){ return _currentForce; }


std::pair<rw::math::Q,rw::math::Q> Robotiq3::getLimitPos(){
    return std::make_pair( rw::math::Q(4,0,0,0,-16), rw::math::Q(4,66,66,66,10) );
}

std::pair<rw::math::Q,rw::math::Q> Robotiq3::getLimitVel(){
    return std::make_pair( rw::math::Q(4,0,0,0,0), rw::math::Q(4,1,1,1,1) );
}

std::pair<rw::math::Q,rw::math::Q> Robotiq3::getLimitCurr(){
    return std::make_pair( rw::math::Q(4,0,0,0,0), rw::math::Q(4,1,1,1,1) );
}

