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

#include "FanucDriver.hpp"

#include "FanucDLLInterface.h"

#include <string>
#include <cmath>
#include <rw/math/Constants.hpp>
#include <rw/common/macros.hpp>

#include <time.h>

const float WAIT_STATE = 0;
const float CNT_STATE =  1;
const float FINE_STATE = 2;
const float CALL_STATE = 3;

const int MIN_SPEED_OVERRIDE = 0;
const int MAX_SPEED_OVERRIDE = 100;
const int MIN_ACC_OVERRIDE = 0;
const int MAX_ACC_OVERRIDE = 500;
const int ERROR_UPDATE_RATE = 20;

using namespace boost::numeric;
using namespace rw::math;
using namespace rwhw;

namespace
{
    long CurrentTimeMs()
    {
        return clock() * 1000 / CLOCKS_PER_SEC;
    }
}

FanucDriver::FanucDriver(std::string ipNr, size_t updateRate)
    :
    _updateRate(updateRate),
    _ipNr(ipNr),
    _isLibraryOpen(false),
    _isConnected(false),
    _qCurrent(Q::zero(6)),
    _qdCurrent(Q::zero(6)),
    _timeStamp(CurrentTimeMs()),
    _nextCmd(IdleCMD),
    _lastCmd(IdleCMD),
    _accCnt(100),
    _accFine(100),
    _speedCnt(100),
    _speedFine(100),
    _globalSpeed(10),
    _accCntChanged(false),
    _accFineChanged(false),
    _speedCntChanged(false),
    _speedFineChanged(false),
    _globalSpeedChanged(false),
    _isCallFinished(true),
    _notifyProgram(false),
    _isCallWaiting(false),
    _error(0.0),
    _updateCnt(0),
    _resetError(false)
{
    _qValCntLast = Q(Q::zero(6));
}

FanucDriver::~FanucDriver()
{

    // remember to clean up
    disconnect();
}

bool FanucDriver::connect(){

    // open the DLL library
    if ( !_isLibraryOpen && !openLibrary()){
        std::cerr << "(FanucDriver) Failed to open library..." << std::endl;
        return false;
    }

    _isLibraryOpen = true;

    // try and connect to the robot controller
    _iHandle = InitInterface((char*)_ipNr.c_str(), _updateRate);

    if(_iHandle == NULL){
        std::cerr << "(FanucDriver) Failed to connect to robot..." << std::endl;
        // remember to clean up
        closeLibrary();
        _isLibraryOpen = false;
        return false;
    }
    _isConnected = true;
    // initialize all register and joint register handles
    _qHomeReg = AddJointRegister(_iHandle,19); // start
    _qCntReg = AddJointRegister(_iHandle,24);
    _qFineReg = AddJointRegister(_iHandle,25);

    //RegisterHandle reg19 = AddRegister(iHandle,19);
    _stateReg = AddRegister(_iHandle,24);
    _notifyReg = AddRegister(_iHandle,25);
    _movFineReg = AddRegister(_iHandle,26);
    _movCntReg = AddRegister(_iHandle,27);
    _accFineReg = AddRegister(_iHandle,28);
    _accCntReg = AddRegister(_iHandle,29);

    _arg1Reg = AddRegister(_iHandle, 60);
    _arg2Reg = AddRegister(_iHandle, 61);
    _arg3Reg = AddRegister(_iHandle, 62);
    _errorReg = AddRegister(_iHandle, 64);

    TJoints joints;
    ReadJoints(_iHandle,&joints);
    joints.q[2] += joints.q[1]; // hack to remove the dependant joints

    WriteJointReg(_qHomeReg, &joints);
    WriteJointReg(_qCntReg, &joints);

    for(int i=0;i<6;i++)
        _qCurrent(i) = joints.q[i]*Deg2Rad;
    return true;
}

void FanucDriver::disconnect(){
    if(_isConnected)
        CloseInterface(_iHandle);
    _isConnected = false;
    if(_isLibraryOpen)
        closeLibrary();
    _isLibraryOpen = false;
}

bool FanucDriver::isConnected(){
    return _isConnected;
}

void FanucDriver::update(){
    if(!_isConnected){
        std::cerr << "The FanucDriver is not connected to the robot controller" << std::endl;
        return;
    }

    // update global speed if necesary
    if(_globalSpeedChanged){
        std::cout << "Global set: " << _globalSpeed << std::endl;
        WriteSpeed(_iHandle, _globalSpeed);
        _globalSpeedChanged = false;
    }

    if( _resetError ){
        _resetError = false;
        _cmdChanged = true;
        _isCallFinished = true;
        _nextCmd = IdleCMD;
        _lastCmd = IdleCMD;
        std::cout << "Err" << std::endl;
        WriteRegister( _errorReg, 0.0 );
        WriteRegister(_stateReg, WAIT_STATE);
        return;
    } else if( ERROR_UPDATE_RATE>_updateCnt){
        _updateCnt = 0;
        ReadRegister( _errorReg, &_error);
    } else {
        _updateCnt++;
    }

    // now update the registers acording to the current command
    switch(_nextCmd){
    case(IdleCMD):
        if(_cmdChanged){
            WriteRegister(_stateReg, WAIT_STATE);
            _cmdChanged = false;
        }
        break;
    case(CntCMD):
    {

        //if( norm_2(_qValCnt-_qValCntLast)>0.01 && norm_2(_qCurrent) ) {
        	//std::cout << "CntCmd" << std::endl;
            _qValCntLast = _qValCnt;
            if( _qCntChanged ){
                TJoints jointTarget;
                for(int i = 0; i<6; i++){
                    jointTarget.q[i] = (float)(_qValCnt[i]*Rad2Deg);
                }
                jointTarget.q[2] -= jointTarget.q[1]; // hack to remove the dependant joints

                WriteJointReg(_qCntReg, &jointTarget);
            }

            //if(_cmdChanged){
                WriteRegister(_stateReg, CNT_STATE);
                _cmdChanged = false;
            //}

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
            //    WriteRegister(_notifyReg, 1.0);
            }
        //} else {
            //std::cout << "not valid!!" << std::endl;
        //}
        _qCntChanged = false;
        break;
    }
    case(FineCMD):

        if( _qFineChanged ){
            TJoints jointTarget;
            for(int i = 0; i<6; i++)
                jointTarget.q[i] = (float)(_qValFine[i]*Rad2Deg);
            jointTarget.q[2] -= jointTarget.q[1]; // hack to remove the dependant joints

            WriteJointReg(_qFineReg, &jointTarget);
        }

        if(_cmdChanged){
            WriteRegister(_stateReg, FINE_STATE);
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
            WriteRegister(_notifyReg, 1.0);
        }
        _qFineChanged = false;
        break;

    case(CallCMD): {
        if( _cmdChanged ){
            _isCallFinished = false;
            _cmdChanged = false;
            _notifyProgram = false;
            _isCallWaiting = false;

            std::cout
                << "Call program: "
                << _progNrToCall
                << "("
                << _callArg[0]
                << " , "
                << _callArg[1]
                << " , "
                << _callArg[2]
                << ")\n";

            WriteRegister(_notifyReg, 1.0 );
            WriteRegister(_arg1Reg, _callArg[0] );
            WriteRegister(_arg2Reg, _callArg[1] );
            WriteRegister(_arg3Reg, _callArg[2] );

            WriteRegister(_stateReg, CALL_STATE+_progNrToCall );
        } else {
            float stateVal,notifyVal;
            ReadRegister(_stateReg, &stateVal );

            ReadRegister(_notifyReg, &notifyVal);
            _isCallWaiting = (notifyVal == 0.0);
            if(_isCallWaiting && _notifyProgram){
                WriteRegister(_notifyReg, 1.0);
            }

            if( stateVal == WAIT_STATE ){
                // finished executing sub program
                _isCallFinished = true;
                _nextCmd = IdleCMD;
            } else if(stateVal!=CALL_STATE+_progNrToCall) {
                WriteRegister(_stateReg, CALL_STATE+_progNrToCall );
            }
        }
    }
    default:
        RW_ASSERT(true);
    }
    _lastCmd = _nextCmd;

    // now read all values from the robot controller
    long lastTimeStamp = _timeStamp;
    Q qLast = _qCurrent;

    // Update joint values
    _qCurrent = getPos();

    // Get time stamp
    _timeStamp = CurrentTimeMs();
    double timeDiff = ((double)(_timeStamp-lastTimeStamp))/1000.0; // convert to seconds

    // calculate velocity
    for(size_t i=0;i<_qCurrent.size();i++){
        double diffval = _qCurrent(i) - qLast(i);
        if( fabs(diffval)>0.001 ) {
            _qdCurrent(i) = diffval/timeDiff;
        } else {
            _qdCurrent(i) = 0.0;
        }
    }
}

Q FanucDriver::getPos()
{
    TJoints joint;
    ReadJoints(_iHandle,&joint);
    joint.q[2] += joint.q[1]; // hack to remove the dependant joints
    ublas::bounded_vector<double,6> result;
    for(int i = 0; i<6; i++)
        result(i)=joint.q[i]*Deg2Rad;
    return Q(result);
}

/******************* THE commands ***************** **************/

void FanucDriver::moveCntQ(Q const &q)
{
    RW_ASSERT(q.size()==6);
    // make sure you don't interupt a running subprogram
    if( !_isCallFinished ){
        std::cout << "MoveCnt: Call blocking!!" << std::endl;
        return;
    }

    _qValCnt = q; // hack to remove the dependant joints
    _qCntChanged = true;
    _nextCmd = CntCMD;
    //if(_nextCmd != _lastCmd)
    _cmdChanged = true;
}

void FanucDriver::moveFineQ(Q const &q)
{
    RW_ASSERT(q.size()==6);
    // make sure you don't interupt a running subprogram

    if( !_isCallFinished ){
        std::cout << "MoveFine: Call blocking!!" << std::endl;
        return;
    }

    _qValFine = q; // hack to remove the dependant joints
    _qFineChanged = true;
    _nextCmd = FineCMD;
    //if(_nextCmd != _lastCmd)
    _cmdChanged = true;
}

bool FanucDriver::isMoveComplete()
{
    return true;
}

/********************** The options for the commands **************/

void FanucDriver::setGlobalSpeed(int speed)
{
    if(speed<0 || speed>100)
        return;
    _globalSpeed = speed;
    _globalSpeedChanged = true;
}

int FanucDriver::getGlobalSpeed()
{
    return _globalSpeed;
}

void FanucDriver::setCntQSpeed(int speed)
{
    if(speed<MIN_SPEED_OVERRIDE || speed>MAX_SPEED_OVERRIDE)
        return;
    _speedCnt = speed;
    _speedCntChanged = true;
}

void FanucDriver::setFineQSpeed(int speed)
{
    if(speed<MIN_SPEED_OVERRIDE || speed>MAX_SPEED_OVERRIDE)
        return;
    _speedFine = speed;
    _speedFineChanged = true;
}

void FanucDriver::setCntQAcc(int acc)
{
    if(acc<MIN_ACC_OVERRIDE || acc>MAX_ACC_OVERRIDE)
        return;
    _accCnt = acc;
    _accCntChanged = true;
}

void FanucDriver::setFineQAcc(int acc)
{
    if(acc<MIN_ACC_OVERRIDE || acc>MAX_ACC_OVERRIDE)
        return;
    _accFine = acc;
    _accFineChanged = true;
}

Q FanucDriver::getQ()
{
    return _qCurrent;
}

Q FanucDriver::getdQ()
{
    return _qdCurrent;
}

void FanucDriver::callRobotProgram(unsigned int progNr)
{
    std::cout << "FanucDriver: CallProgram - " << progNr << std::endl;
    // make sure you don't interupt a running subprogram
    if( !_isCallFinished ){
        std::cout << "PROGRAM is allready running!! "<< std::endl;
        return;
    }
    _isCallFinished = false;
    _progNrToCall = progNr;
    _nextCmd = CallCMD;

    if (_nextCmd != _lastCmd)
        _cmdChanged = true;
}

void FanucDriver::setCallArguments(float arg1, float arg2, float arg3)
{
    std::cout << "Set arg: " << arg1 << " , " << arg2 << " , " << arg3 << std::endl;
    _callArg[0] = arg1;
    _callArg[1] = arg2;
    _callArg[2] = arg3;
}

bool FanucDriver::isCallFinished()
{
    return _isCallFinished;
}

bool FanucDriver::notifyProgram()
{
    if(_isCallFinished)
        return false;
    _notifyProgram = true;
    return true;
}

bool FanucDriver::isCallWaiting()
{
    if(_isCallFinished)
        return false;
    return _isCallWaiting;
}
