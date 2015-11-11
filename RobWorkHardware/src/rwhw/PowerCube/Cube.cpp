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

#include "Cube.hpp"

#include <rw/common/TimerUtil.hpp>
#include <rw/common/macros.hpp>

#include <iostream>
#include <vector>

#include <stdio.h>
#include <cmath>

using namespace rw::common;
using namespace rwhw;

inline double round(double d) { return floor(d + 0.5); }

namespace {
/*
    int cmdIdAckp( int _moduleId ) {
        return PCUBE_CANID_CMDACK + _moduleId;
    }

    int cmdIdGetp( int _moduleId ) {
        return PCUBE_CANID_CMDGET + _moduleId;
    }

    int cmdIdPutp( int _moduleId ) {
        return PCUBE_CANID_CMDPUT + _moduleId;
    }

    int Cmd::ALL {
        return PCUBE_CANID_CMDALL;
    }
*/
    void emitp(const Cmd& cmd, CubePort *port){
        if (!port->broadcast( cmd )) {
            RW_THROW("Error writing canport!!");
        }
    }

    void emitp(const Cmd& cmd, CubePort *port, int modulenr){
        if (!port->write( cmd, modulenr)) {
            RW_THROW("Error writing canport!!");
        }
    }
}

Cube::Cube(int moduleId, CubePort* port):
    _moduleId( moduleId ),
    _port( port )
{}

Cube::~Cube()
{}


bool Cube::ping() {
	return Cube::ping(_moduleId, _port);
}


// global commands
bool Cube::ping( int moduleNr, CubePort* port )
{
    // request version nr from cube with moduleNr
/*
    emitp( Cmd( Cmd::PUT,
    	  PCubeProtocol::makeData(0x01)),
          port,
          moduleNr);
*/
    emitp( Cmd( Cmd::GET,
          PCubeProtocol::makeData(PCUBE_GetExtended, PCUBE_DefCubeVersion) ),
          port,
          moduleNr);

    TimerUtil::sleepMs(100);

    CubePort::Message msg;
    if( port->read( msg ) )
        return true;

    // if no ack is returned then the cube is not connected.
    return false;
}

std::vector<Cube*> Cube::getCubes(size_t from, size_t to, CubePort* port ){
    std::vector<Cube*> cubes;
    for(size_t i=from; i<to; i++ ){
        bool connected = Cube::ping(i, port);
        if( !connected )
            continue;
        std::cout << std::endl << "Cube with id: " << i << " is connected." << std::endl;
        Cube *cube = Cube::getCubeAt(i, port);
        if(cube==NULL)
        	RW_THROW("Cube cannot be initialized!!");
        cubes.push_back(cube);
    }
	return cubes;
}

Cube* Cube::getCubeAt(int moduleNr , CubePort* port )
{
    if( Cube::ping( moduleNr , port ) ){
        return new Cube( moduleNr, port);
    }
    return NULL;
}

void Cube::resetAllCmd(CubePort* port )
{
    emitp( Cmd( Cmd::ALL,
          PCubeProtocol::makeData(PCUBE_ResetAll) ), port );
}

void Cube::homeAllCmd(CubePort* port )
{
    emitp( Cmd( Cmd::ALL,
          PCubeProtocol::makeData(PCUBE_HomeAll) ), port );
}

void Cube::haltAllCmd(CubePort* port )
{
    emitp( Cmd( Cmd::ALL,
          PCubeProtocol::makeData(PCUBE_HaltAll) ), port );
}
// watchdog port, _moduleId )h all
void Cube::watchdogAllCmd(CubePort* port )
{
    emitp( Cmd( Cmd::ALL,
          PCubeProtocol::makeData(PCUBE_WatchdogRefreshAll) ), port);
}

void Cube::setBaudrateAll(CubeBaudrate rate, CubePort* port)
{
        emitp( Cmd( Cmd::ALL,
            PCubeProtocol::makeData(PCUBE_SetBaudAll, rate) ), port );
}

void Cube::savePosAllCmd(CubePort* port )
{
    emitp( Cmd( Cmd::ALL,
          PCubeProtocol::makeData(PCUBE_SavePosAll) ), port );
}

void Cube::syncMotionAllCmd(CubePort* port )
{
    emitp( Cmd( Cmd::ALL,
          PCubeProtocol::makeData(PCUBE_SyncMotionAll) ), port );

}
// Reset, home and halt commands.

bool Cube::resetCmd()
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(0)) );
    return ack( Cmd(Cmd::ACK, PCubeProtocol::makeData(0)) );
}

bool Cube::homeCmd()
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(0x01)) );
    return ack( Cmd(Cmd::ACK, PCubeProtocol::makeData(0x01)) );
}

void Cube::haltCmd()
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(0x02)) );
    emitCmd( Cmd(Cmd::ACK, PCubeProtocol::makeData(0x02)) );
}

// Set motion commands.

// Ramp to the position val.
bool Cube::moveRampCmd(float val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_MODE, val)) );
    return ack( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_MODE, 0x64)) );
}

// The time is in seconds.
bool Cube::moveStepCmd(float pos, double time)
{
    // Convert to ms.
    const int ms = (int)round(time * 1000);
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_MODE, pos, ms)) );
    return ack( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_MODE, 0x64)) );
}

// Velocity in m/s.
bool Cube::moveVelCmd(float val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_MODE, val)) );
    return ack( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_MODE, 0x64)) );
}

// Current in Ampere.
bool  Cube::moveCurCmd(float val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_MODE, val)) );
    return ack( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_MODE, 0x64)) );
}

bool Cube::moveRampTicksCmd(int val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_MODE, val)) );
    return ack( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_MODE, 0x64)) );
}

bool Cube::moveStepTicksCmd(int pos, double time)
{
    const int ms = (int)round(time * 1000);
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_MODE, pos, ms)) );
    return ack( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_MODE, 0x64)) );
}

// Encoder ticks per second.
bool Cube::moveVelTicksCmd(int val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_MODE, val)) );
    return ack( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_MODE, 0x64)) );
}

// Current in ??
bool Cube::moveCurTicksCmd(int val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_MODE, val)) );
    return ack( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_MODE, 0x64)) );
}

/*
  Positive velocities open the gripper and negative velocities close
  the gripper.

  Positive currents open the gripper and negative currents close the
  gripper.
*/
// Ramp to the position val.
float Cube::moveRampExtCmd(float val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_ACK, val)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_ACK)),msg ) )
        return PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

//Bogild
Cube::CubeExtAckData Cube::moveRampExtCmdWithState(float val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_ACK, val)) );
    CubePort::Message msg;
	CubeExtAckData data;
	if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_ACK)),msg ) ){
		//printf("can %x %x %x %x  \n",msg.data[2],msg.data[3],msg.data[4],msg.data[5]);
		data.position = PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
		data.state = msg.data[6];
		data.dio = msg.data[7];
        return data;
	}
    else
    	RW_THROW("No acknowledge received!");
	return data; // should not end here..
}

// The time is in seconds.
float Cube::moveStepExtCmd(float pos, double time)
{
    // Convert to ms.
    const int ms = (int)round(time * 1000);
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_ACK, pos, ms)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_ACK)),msg ) )
        return PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

// Velocity in m/s.
float Cube::moveVelExtCmd(float val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_ACK, val)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_ACK)),msg ) )
        return PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

//Bogild
Cube::CubeExtAckData Cube::moveVelExtCmdWithState(float val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_ACK, val)) );
    CubePort::Message msg;
	CubeExtAckData data;
	if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_ACK)),msg ) ){
		data.position = PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
		data.state = msg.data[6];
		data.dio = msg.data[7];
        return data;
	}
    else
    	RW_THROW("No acknowledge received!");
	return data; // should not end here..
}


// Current in Ampere.
float Cube::moveCurExtCmd(float val)
{
    emitCmd(Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_ACK, val)));
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_ACK)),msg ) )
        return PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

int Cube::moveRampTicksExtCmd(int val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_ACK, val)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_ACK)),msg ) )
        return PCubeProtocol::toInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

int Cube::moveStepTicksExtCmd(int pos, double time)
{
    const int ms = (int)round(time * 1000);
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_ACK, pos, ms)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_ACK)), msg ) )
        return PCubeProtocol::toInt(msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

// Encoder ticks per second.
int Cube::moveVelTicksExtCmd(int val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_ACK, val)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_ACK)), msg ) )
        return PCubeProtocol::toInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

// Current in ??
int Cube::moveCurTicksExtCmd(int val)
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_ACK, val)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_ACK)), msg) )
        return PCubeProtocol::toInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

// Bogild
bool Cube::setConfig( unsigned int paramval ){
	emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_Config , paramval)) );
	CubePort::Message msg;
	if( !ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_Config)), msg) ) {
		RW_WARN("SET command did not function correct!!");
		return false;
	}
	return true;
}

// Bogild
/*void Cube::setTargetVel( float paramval ){
	emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_TargetVel , paramval)) );
	io::CubePort::Message msg;
	if( !ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_Config)), msg) )
    	RW_WARN("SET command did not function correct!!");
}

// Bogild
void Cube::setTargetAcc( float paramval ){
	emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_TargetAcc , paramval)) );
	io::CubePort::Message msg;
	if( !ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_Config)), msg) )
    	RW_WARN("SET command did not function correct!!");
}*/


bool Cube::setFloatParam( unsigned char paramid, float paramval )
{
    emitCmd( Cmd(Cmd::PUT, PCubeProtocol::makeData(PCUBE_SetExtended, paramid, paramval)) );
    CubePort::Message msg;
    if( !ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_SetExtended, paramid)), msg) ) {
    	RW_WARN("SET command did not function correct!!");
    	return false;
    }
    return true;
}


float Cube::getFloatParam( unsigned char paramid )
{
    emitCmd( Cmd(Cmd::GET, PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

int Cube::getInt32Param( unsigned char paramid ){
    emitCmd( Cmd(Cmd::GET, PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::toInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

int Cube::getInt16Param( unsigned char paramid ){
    emitCmd( Cmd(Cmd::GET, PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::toInt( 0, 0, msg.data[2], msg.data[3]);
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

char Cube::getInt8Param( unsigned char paramid ){
    emitCmd( Cmd(Cmd::GET, PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    CubePort::Message msg;
    if( ackext( Cmd(Cmd::ACK, PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::toInt( 0, 0, 0, msg.data[2]);
    else
    	RW_THROW("No acknowledge received!");
    return 0; // should not end here..
}

bool Cube::ackext(const Cmd& cmd, CubePort::Message& msg)
{
    int sleepCnt=0;
    while( !getPort().read( msg ) ) {
        TimerUtil::sleepMs(1);
        if( sleepCnt++ > 12 ) {
        	RW_THROW("Ack error: timeout!");
            return false;
        }
    }

    if( msg.msgType != CubePort::Message::ACK ){
        RW_THROW("Ack error: received bad id!");
        return false;
    }
/*
    for(int i=0; i < (int)cmd.data.size(); i++){
        if( msg.data[i] != cmd.data[i] ){
            RW_WARN("Ack error: received bad data!");
            return false;
        }
    }
*/
    return true;
}

bool Cube::ack(const Cmd& cmd)
{
    CubePort::Message msg;
    int sleepCnt=0;
	while( !getPort().read( msg ) ){
        TimerUtil::sleepUs(1);
        if( sleepCnt++ > 12 ) {
            RW_WARN("Ack error: timeout!");
            return false;
        }
    }

	if( msg.msgType != CubePort::Message::ACK ){
    	RW_WARN("Ack error: received bad id!" );
        return false;
    }

/*    std::cout << "ACK MSG: " << msg.id ;
    for( int i=0; i<msg.length; i++)
        std::cout << "["<<(int)msg.data[i]<<"]";
    std::cout << std::endl;
*/
    return true;
}

void Cube::emitCmd(const Cmd& cmd)
{
    getPort().write(cmd, _moduleId);
}
