#include "Robotiq3.hpp"

using namespace rwhw;
using namespace rw::math;
using namespace rw::common;

Robotiq3::Robotiq3():
    Robotiq(rw::math::Q(4,0,0,0,0),
            rw::math::Q(4,0,0,0,0),
            rw::math::Q(4,0,0,0,0),
            rw::math::Q(4,0xFF,0xFF,0xFF,0xFF),
            rw::math::Q(4,0xFF,0xFF,0xFF,0xFF),
            4),
    _statusDataValid(false)
{
}

std::pair<rw::math::Q,rw::math::Q> Robotiq3::getLimitPos(){
    return std::make_pair( rw::math::Q(4,0,0,0,0), rw::math::Q(4,0xFF,0xFF,0xFF,0xFF) );
}

std::pair<rw::math::Q,rw::math::Q> Robotiq3::getLimitVel(){
    return std::make_pair( rw::math::Q(4,0,0,0,0), rw::math::Q(4,0xFF,0xFF,0xFF,0xFF) );
}

std::pair<rw::math::Q,rw::math::Q> Robotiq3::getLimitForce(){
    return std::make_pair( rw::math::Q(4,0,0,0,0), rw::math::Q(4,0xFF,0xFF,0xFF,0xFF) );
}

bool Robotiq3::isGripperInReset() {
    return _gripperStatus.data.gIMC != 0;
}

bool Robotiq3::isGripperInActivationProcess() {
    return (_gripperStatus.data.gIMC!=3 || _gripperStatus.data.gACT==0);
}

double Robotiq3::getVelocityInMetersPerSecFromTicks( const int ticks) const {
    // ticks are 0..255 (as coming back from hand)
    // return in meters per second
    return (22.0 + ticks * (110.0-22.0)/255.0)/1000.0;
}

int Robotiq3::getTicksFromVelocityInMetersPerSec( const double velocity) const {
    // velocity in meters per second
    // return ticks are 0..255 (as sent to hand)
    return ((velocity*1000.0)-22.0)*255.0 / (110.0-22.0);
}

double Robotiq3::getApproximateForceInNewtonFromTicks( const int ticks) const {
    // ticks are 0..255 (as coming back from hand)
    // return in Newton
    return 15.0 + ticks * (60.0-15.0)/255.0;
}

int Robotiq3::getApproximateTicksFromForceInNewton( const double force) const {
    // force in Newton
    // return ticks are 0..255 (as sent to hand)
    return (force - 15.0)*255.0/(60.0-15.0);
}

ModbusPackage Robotiq3::getMoveCMDRequestPackage(const rw::math::Q & target) const {
    ModbusPackage package;

    setReg(package.header.data.functionCode, FC16);
    setReg(package.header.data.length, 2 + 5 + 16);

    ActionRequest actreq;
    actreq.value = 0x00;
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

    cmd.data._posA_req = toVal8(target[0]);
    cmd.data._posB_req = toVal8(target[1]);
    cmd.data._posC_req = toVal8(target[2]);
    cmd.data._posScissor_req = toVal8(target[3]);

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

    return package;
}

ModbusPackage Robotiq3::getAllStatusCMDRequestPackage() const {
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

    return package;
}

void Robotiq3::validateStatusPackageAndUpdateState(const ModbusPackage& package) {
    // number of data bytes to use, should be 16
    boost::uint16_t n;
    getReg(package.header.data.length, n);
    if( package.data[0] != 16 || n != 16+3)
        RW_THROW("answer should contain 16 bytes and not "<< (int) package.data[0] << " and the length should be 19 instead of " << n);

    // now read everything into internal variables
    for(int i=0;i<16;i++){
        _status.value[i] = package.data[1+i];
    }

    _gripperStatus.value = package.data[1];
    _objectStatus.value = package.data[2];
    _faultStatus.value = package.data[3];

    _statusDataValid = true;

    std::pair<Q,Q> lim = getLimitPos();

    _currentQ(0) = _status.data._posA;
    _currentQ(1) = _status.data._posB;
    _currentQ(2) = _status.data._posC;
    _currentQ(3) = _status.data._posScissor;

    _currentCurrent(0)  = _status.data._currentA;
    _currentCurrent(1)  = _status.data._currentB;
    _currentCurrent(2)  = _status.data._currentC;
    _currentCurrent(3)  = _status.data._currentScissor;

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

ModbusPackage Robotiq3::getStopCMDRequestPackage() const {
    ModbusPackage package;

    setReg(package.header.data.functionCode, FC16);
    setReg(package.header.data.length, 2+5+2);
    setReg(package.header.data.unitID, 2);

    ActionRequest actreq;
    actreq.value = 0x00;
    actreq.data.rACT = 1;

    GripperOptions gripopt;
    gripopt.value = 0x00;
    gripopt.data.rICF = 1; // individual finger control
    gripopt.data.rICS = 1; // individual control of scissor. disable mode selection

    // register start address
    package.data[0] = 0x00;
    package.data[1] = 0x00;

    // number of registers
    package.data[2] = 0x00;
    package.data[3] = 0x01;
    // number data bytes to follow
    package.data[4] = 0x02;

    package.data[5] = actreq.value;
    package.data[6] = gripopt.value;

    return package;
}

void Robotiq3::validateStopCMDResponseMessage(const ModbusPackage & answer) const {
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

ModbusPackage Robotiq3::getActivateRequestPackage() const {
    ModbusPackage package;

    setReg(package.header.data.functionCode, FC16);
    setReg(package.header.data.length, 13);
    setReg(package.header.data.unitID, 2);

    // register start address
    package.data[0] = 0x00;
    package.data[1] = 0x00;

    // number of registers
    package.data[2] = 0x00;
    package.data[3] = 0x03;
    // number data bytes to follow
    package.data[4] = 0x06;

    package.data[5] = 0x01; // action request
    package.data[6] = 0x0c; // gripper options

    package.data[7] = 0x00; // register 0x0001
    package.data[8] = 0x00;

    package.data[9] = 0x00; // register 0x0002
    package.data[10] = 0x00;


    ActionRequest actreq;
    actreq.value = 0x00;
    actreq.data.rACT = 1;

    GripperOptions gripopt;
    gripopt.value = 0x00;
//    gripopt.data.rICF = 1; // individual finger control
//    gripopt.data.rICS = 1; // individual control of scissor. disable mode selection

    ActionRequestCMD cmd;
    cmd.data._actionRequest = actreq.value;
    cmd.data._gripperOptions = gripopt.value;
    cmd.data._gripperOptions2 = 0x00;

    cmd.data._posA_req = 0x00;
    cmd.data._speedA = 0x00;
    cmd.data._forceA = 0x00;


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

