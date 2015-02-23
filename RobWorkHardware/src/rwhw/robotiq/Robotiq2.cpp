#include "Robotiq2.hpp"

using namespace rwhw;
using namespace rw::math;
using namespace rw::common;

Robotiq2::Robotiq2():
    Robotiq(rw::math::Q(1,0.0),
            rw::math::Q(1,0.0),
            rw::math::Q(1,0.0),
            rw::math::Q(1,255.0),
            rw::math::Q(1,255.0)),
    _statusDataValid(false)
{
}

std::pair<rw::math::Q,rw::math::Q> Robotiq2::getLimitPos(){
    return std::make_pair( rw::math::Q(1,0.0), rw::math::Q(1,255.0) );
}

std::pair<rw::math::Q,rw::math::Q> Robotiq2::getLimitVel(){
    return std::make_pair( rw::math::Q(1,0.0), rw::math::Q(1,255.0) );
}

std::pair<rw::math::Q,rw::math::Q> Robotiq2::getLimitForce(){
    return std::make_pair( rw::math::Q(1,0.0), rw::math::Q(1,255.0) );
}

bool Robotiq2::isGripperInReset() {
    return _gripperStatus.data.gACT == 0;
}

bool Robotiq2::isGripperInActivationProcess() {
    return (_gripperStatus.data.gSTA!=3 || _gripperStatus.data.gACT==0);
}

double Robotiq2::getFingerDistanceInMetersFromTicks( const int ticks) const {
    // ticks are 0..255 (as coming back from hand)
    // return in meters
    if (ticks <= 13) {
        return 87.0 / 1000.0;
    } else if(ticks >= 230) {
        return 0.0;
    } else {
        return (87.0 - 87.0/(230.0-13.0) * (ticks-13.0))/1000.0;
    }
}

int Robotiq2::getTicksFromFingerDistanceInMeters( const double distance) const {
    // distance in meters
    // return ticks are 0..255 (as sent to hand)
    return ((1.0 - (distance*1000.0)/87.0) * (230.0-13.0)) + 13;
}

double Robotiq2::getVelocityInMetersPerSecFromTicks( const int ticks) const {
    // ticks are 0..255 (as coming back from hand)
    // return in meters per second
    return (13.0 + ticks * (100.0-13.0)/255.0)/1000.0;
}

int Robotiq2::getTicksFromVelocityInMetersPerSec( const double velocity) const {
    // velocity in millimeters per second
    // return ticks are 0..255 (as sent to hand)
    return ((velocity*1000.0)-13.0)*255.0 / (100.0-13.0);
}

double Robotiq2::getApproximateForceInNewtonFromTicks( const int ticks) const {
    // ticks are 0..255 (as coming back from hand)
    // return in Newton
    return 30.0 + ticks * (100.0-30.0)/255.0;
}

int Robotiq2::getApproximateTicksFromForceInNewton( const double force) const {
    // force in Newton
    // return ticks are 0..255 (as sent to hand)
    return (force - 30.0)*255.0/(100.0-30.0);
}

ModbusPackage Robotiq2::getMoveCMDRequestPackage(const rw::math::Q & target) const {
    ModbusPackage package;

    setReg(package.header.data.functionCode, FC16);
    setReg(package.header.data.length, 2 + 5 + 6);

    ActionRequest actreq;
    actreq.value = 0x00;
    actreq.data.rACT = 1;
    actreq.data.rGTO = 1;
    actreq.data.rATR = 0; // Normal, not automatic release

    ActionRequestCMD cmd;
    cmd.data._actionRequest = actreq.value;
    cmd.data._reserved1 = 0x00;
    cmd.data._reserved2 = 0x00;
    cmd.data._pos_req = toVal8(_target[0]);
    cmd.data._speed = toVal8(_speed[0]);
    cmd.data._force = toVal8(_force[0]);

    // register start address
    package.data[0] = 0x00;
    package.data[1] = 0x00;
    // number of registers
    package.data[2] = 0x00;
    package.data[3] = 0x03;
    // number data to follow
    package.data[4] = 6;

    // now come the positions, speed and forces
    for(int i=0;i<6;i++){
        package.data[5+i] = cmd.value[i];
    }

    return package;
}

ModbusPackage Robotiq2::getAllStatusCMDRequestPackage() const {
    ModbusPackage package;

    setReg(package.header.data.functionCode, FC04);
    setReg(package.header.data.length, 6);
    setReg(package.header.data.unitID, 2);

    // register start address
    package.data[0] = 0x00;
    package.data[1] = 0x00;
    // number of registers
    package.data[2] = 0x00;
    package.data[3] = 0x03;

    return package;
}

void Robotiq2::validateStatusPackageAndUpdateState(const ModbusPackage& package) {
    // number of data bytes to use, should be 6
    boost::uint16_t n;
    getReg(package.header.data.length, n);
    if( package.data[0] != 6 || n != 6+3)
        RW_THROW("answer should contain 6 bytes and not "<< (int) package.data[0] << " and the length should be 9 instead of " << n);

    // now read everything into internal variables
    GripperStatusAll status;
    for(int i=0;i<6;i++){
        status.value[i] = package.data[1+i];
    }

    _gripperStatus.value = status.data._gripperStatus;
    _faultStatus.value = status.data._faultStatus;
    _statusDataValid = true;

    _currentQ(0) = status.data._pos;
    _currentCurrent(0)  = status.data._current;

    // todo Move this to trace level instead of debug once trace is implemented
    if (Log::log().isEnabled(Log::Debug)) {
        std::stringstream traceMessage;
        traceMessage << "Status: ";
        traceMessage.fill('0');
        for(int i=0;i<6;i++){
            traceMessage << std::setw(2) << std::hex << (unsigned int)status.value[i] << " ";
        }
        traceMessage << std::endl;
        traceMessage << "gSTA:"<< _gripperStatus.data.gSTA << " gACT:"<<_gripperStatus.data.gACT;
        RW_LOG_DEBUG(traceMessage.str());
    }
}

ModbusPackage Robotiq2::getStopCMDRequestPackage() const {
    ModbusPackage package;

    setReg(package.header.data.functionCode, FC16);
    setReg(package.header.data.length, 2+5+2);
    setReg(package.header.data.unitID, 2);

    ActionRequest actreq;
    actreq.value = 0x00;
    actreq.data.rACT = 1;

    // register start address
    package.data[0] = 0x00;
    package.data[1] = 0x00;

    // number of registers
    package.data[2] = 0x00;
    package.data[3] = 0x01;
    // number data bytes to follow
    package.data[4] = 0x02;

    package.data[5] = actreq.value;
    package.data[6] = 0x00;

    return package;
}

void Robotiq2::validateStopCMDResponseMessage(const ModbusPackage & answer) const {
    // check if response is correct
    boost::uint16_t n;
    getReg(answer.header.data.length, n);
    if (n != 6 ||
        answer.data[0] != 0x00 ||
        answer.data[1] != 0x00 ||
        answer.data[2] != 0x00 ||
        answer.data[3] != 0x01)
        RW_THROW("Received message is wrong size (is " << n << " should be " << 6 << ") or wrong content");
}

ModbusPackage Robotiq2::getActivateRequestPackage() const {
    ModbusPackage package;

    setReg(package.header.data.functionCode, FC16);
    setReg(package.header.data.length, 2+5+6);
    setReg(package.header.data.unitID, 2);

    ActionRequest actreq;
    actreq.value = 0x00;
    actreq.data.rACT = 1;

    ActionRequestCMD cmd;
    cmd.data._actionRequest = actreq.value;
    cmd.data._reserved1 = 0x00;
    cmd.data._reserved2 = 0x00;
    cmd.data._pos_req = 0x00;
    cmd.data._speed = 0x00;
    cmd.data._force = 0x00;

    // register start address
    package.data[0] = 0x00;
    package.data[1] = 0x00;
    // number of registers
    package.data[2] = 0x00;
    package.data[3] = 0x03;
    // number data to follow
    package.data[4] = 6;

    // now come the request data, positions, speed and forces
    for(int i=0;i<6;i++){
        package.data[5+i] = cmd.value[i];
    }

    return package;
}
