/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "FanucVirtual.hpp"

#include <rw/common/Timer.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/models/Joint.hpp>

const int MIN_SPEED_OVERRIDE = 0;
const int MAX_SPEED_OVERRIDE = 100;
const int MIN_ACC_OVERRIDE = 0;
const int MAX_ACC_OVERRIDE = 500;

using namespace rwhw;
using namespace rwhw::fanuc;
using namespace rw::common;
using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;
using namespace boost::numeric;

namespace
{
    long currentTimeMs()
    {
        return TimerUtil::currentTimeMs();
    }
}

FanucVirtual::FanucVirtual(
    rw::models::SerialDevice* fanucModel,
    const State& state)
    :
    _model(fanucModel), _state(state), _isConnected(false),
    _qCurrent(Q::zero(6)),
    _dqCurrent(Q::zero(6)),
    _timeStamp(currentTimeMs()),
    _nextCmd(IdleCMD), _lastCmd(IdleCMD),
    _accCnt(100), _accFine(100), _speedCnt(100), _speedFine(100), _globalSpeed(10),
    _accCntChanged(false), _accFineChanged(false), _speedCntChanged(false),
    _speedFineChanged(false), _globalSpeedChanged(false)
{
    // initialize Velocity Ramp Profile using the Device
    typedef std::vector<std::pair<double,double> > VecPairs;
    VecPairs posLimit;
    VecPairs velLimit;
    VecPairs accLimit;

    int dof = _model->getDOF();

    for(int i=0;i<dof;i++){
        posLimit.push_back(std::make_pair(_model->getBounds().first[i], _model->getBounds().second[i]));
        std::pair<double, double> tmp;
        tmp.first = _model->getVelocityLimits()[i];
        tmp.second = -tmp.first;
        velLimit.push_back( tmp );
        tmp.first = _model->getAccelerationLimits()[i];
        tmp.second = -tmp.first;
        accLimit.push_back( tmp );
    }
    _velProfile = new VelRampProfile(posLimit,velLimit,accLimit);
}

FanucVirtual::~FanucVirtual()
{
    delete _velProfile;
}

bool FanucVirtual::connect(){

    _isConnected = true;

    _qCurrent = _model->getQ(_state);
    _qGoal = _qCurrent;
    return true;
}

void FanucVirtual::disconnect(){
    _isConnected = false;
}

bool FanucVirtual::isConnected(){
    return _isConnected;
}

/******************* THE commands *******************************/

void FanucVirtual::moveCntQ(Q const &q){
    RW_ASSERT(q.size()==6);
    _qValCnt = q;
    _qCntChanged = true;
    _nextCmd = CntCMD;
    if(_nextCmd != _lastCmd)
        _cmdChanged = true;
}

void FanucVirtual::moveFineQ(Q const &q){
    RW_ASSERT(q.size()==6);
    _qValFine = q;
    _qFineChanged = true;
    _nextCmd = FineCMD;
    if(_nextCmd != _lastCmd)
        _cmdChanged = true;
    // std::cout << "MOVE FINE CMD" << std::endl;
}

bool FanucVirtual::isMoveComplete(){
    return true;
}

void FanucVirtual::setGlobalSpeed(int speed){
    if(speed<0 || speed>100)
        return;
    _globalSpeed = speed;
    _globalSpeedChanged = true;
}

int FanucVirtual::getGlobalSpeed(){
    return _globalSpeed;
}

void FanucVirtual::setCntQSpeed(int speed){
    if(speed<MIN_SPEED_OVERRIDE || speed>MAX_SPEED_OVERRIDE)
        return;
    _speedCnt = speed;
    _speedCntChanged = true;
}

void FanucVirtual::setFineQSpeed(int speed){
    if(speed<MIN_SPEED_OVERRIDE || speed>MAX_SPEED_OVERRIDE)
        return;
    _speedFine = speed;
    _speedFineChanged = true;
}

void FanucVirtual::setCntQAcc(int acc){
    if(acc<MIN_ACC_OVERRIDE || acc>MAX_ACC_OVERRIDE)
        return;
    _accCnt = acc;
    _accCntChanged = true;
}

void FanucVirtual::setFineQAcc(int acc){
    if(acc<MIN_ACC_OVERRIDE || acc>MAX_ACC_OVERRIDE)
        return;
    _accFine = acc;
    _accFineChanged = true;
}

Q FanucVirtual::getQ(){
    return _qCurrent;
}

Q FanucVirtual::getdQ(){
    return _dqCurrent;
}

void FanucVirtual::update(){
    if(!_isConnected){
        std::cerr << "The FanucDriver is not connected to the robot controller" << std::endl;
        return;
    }
    long time = currentTimeMs();
    double timediff = ((double)(time-_timeStamp))/1000.0;
    _timeStamp = time;
    //timediff = 0.05;
    // update global speed if necesary
    if(_globalSpeedChanged){
        //_velProfile->setSpeedPercent(_globalSpeed);
        _globalSpeedChanged = false;
    }
    // now update the registers acording to the current command
    switch(_nextCmd){
    case(IdleCMD):
        if(_cmdChanged){
            _cmdChanged = false;
        }
        break;
    case(CntCMD):
    {
        if( _qCntChanged ){
            //WriteJointReg(_qCntReg, &jointTarget);
/*            std::cout << "Cnt cmd: " << _qValCnt << std::endl;
              std::cout << " " << _qValCnt  << std::endl
              << " " << _qCurrent  << std::endl
              << " " << _dqCurrent  << std::endl
              << " " << timediff  << std::endl;
*/
            _qGoal = _qValCnt;
        }

        if(_cmdChanged){
            _cmdChanged = false;
        }

        if(_speedCntChanged){
            // TODO: update cnt speed
            _speedCntChanged = false;
        }

        if(_accCntChanged){
            // TODO: update cnt acc
            _accCntChanged = false;
        }

        // TODO: this last notification should be optimized away
        if( _qCntChanged  ){
            //
            _qCntChanged = false;
        }


        break;
    }
    case(FineCMD):

        if( _qFineChanged ){
            /*std::cout << "Fine cmd: " << _qValFine << std::endl;
              std::cout << " " << _qValFine  << std::endl
              << " " << _qCurrent  << std::endl
              << " " << _dqCurrent  << std::endl
              << " " << timediff  << std::endl;
            */
            _qGoal = _qValFine;
        }

        if(_cmdChanged){
            _cmdChanged = false;
        }

        if(_speedFineChanged){
            // TODO: update cnt speed
            _speedFineChanged = false;
        }

        if(_accFineChanged){
            // TODO: update cnt acc
            _accFineChanged = false;
        }

        if(_qFineChanged ){
            _qFineChanged = false;
        }

        break;
    default:
        RW_ASSERT(true);
    }
    _dqCurrent = _velProfile->getVelocity(_qGoal,_qCurrent,_dqCurrent, timediff );
    _qCurrent += _dqCurrent*timediff;
    //std::cout << "new curr vel: " << _dqCurrent << std::endl;
    //std::cout << "new curr pos: " << _qCurrent << std::endl;
    _model->setQ(_qCurrent, _state);
    _lastCmd = _nextCmd;
}

void FanucVirtual::callRobotProgram(unsigned int progNr)
{
    std::cout
        << "Robot program nr. "
        << progNr
        << " called!\n"
        << "  --- with arg ("
        << _callArg[0]
        << ","
        << _callArg[1]
        << ","
        << _callArg[2]
        << ")\n";
}

void FanucVirtual::setCallArguments(float arg1, float arg2, float arg3)
{
    _callArg[0] = arg1;
    _callArg[1] = arg2;
    _callArg[2] = arg3;
}

bool FanucVirtual::isCallFinished()
{
    return true;
}
