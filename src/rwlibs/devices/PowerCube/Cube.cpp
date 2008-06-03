//#include "include/m5apiw32.h"

#include "Cube.hpp"
#include <rwlibs/io/canbus/CanPort.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/macros.hpp>

#include <iostream>
#include <vector>

#include <stdio.h>
#include <cmath>

using namespace rw::common;
using namespace rwlibs;
using namespace rwlibs::devices;

inline double round(double d) { return floor(d + 0.5); }

namespace {

    int cmdIdAckp( int _moduleId ) {
        return PCUBE_CANID_CMDACK + _moduleId;
    }

    int cmdIdGetp( int _moduleId ) {
        return PCUBE_CANID_CMDGET + _moduleId;
    }

    int cmdIdPutp( int _moduleId ) {
        return PCUBE_CANID_CMDPUT + _moduleId;
    }

    int cmdIdAll() {
        return PCUBE_CANID_CMDALL;
    }

    void emitp(const Cmd& cmd, rwlibs::io::CanPort *port)
    {
        if (!port->write( cmd.id, cmd.data )) {
            RW_THROW("Error writing canport!!");
        }
    }
}

Cube::Cube(int moduleId, io::CanPort* port):
    _moduleId( moduleId ),
    _port( port )
{}

Cube::~Cube()
{}

// global commands
bool Cube::isCubeConnected( int moduleNr, io::CanPort* port )
{
    // request version nr from cube with moduleNr

    emitp( Cmd( cmdIdGetp(moduleNr),
          PCubeProtocol::makeData(PCUBE_GetExtended, PCUBE_DefCubeVersion) ), port );
     
	/*emitp( Cmd( cmdIdPutp(moduleNr),
			PCubeProtocol::makeData(0x01) ), port );
	*/
    TimerUtil::sleepMs(20);

    io::CanPort::CanMessage msg;
    if( port->read( msg ) )
        return true;

    // if no ack is returned then the cube is not connected.
    return false;
}

std::vector<Cube*> Cube::getCubes(size_t from, size_t to, io::CanPort* port ){
    std::vector<Cube*> cubes;
    for(size_t i=from; i<to; i++ ){
        bool connected = Cube::isCubeConnected(i, port);
        if( !connected )
            continue;
        //std::cout << std::endl << "Cube with id: " << i << " is connected." << std::endl;
        Cube *cube = Cube::getCubeAt(i, port);
        cubes.push_back(cube);
    }
	return cubes;
}

Cube* Cube::getCubeAt( int moduleNr , io::CanPort* port )
{
    if( isCubeConnected( moduleNr , port ) ){
        return new Cube( moduleNr, port);
    }
    return NULL;
}

void Cube::resetAllCmd( io::CanPort* port )
{
    emitp( Cmd( cmdIdAll(),
          PCubeProtocol::makeData(PCUBE_ResetAll) ), port );
}

void Cube::homeAllCmd( io::CanPort* port )
{
    emitp( Cmd( cmdIdAll(),
          PCubeProtocol::makeData(PCUBE_HomeAll) ), port );
}

void Cube::haltAllCmd( io::CanPort* port )
{
    emitp( Cmd( cmdIdAll(),
          PCubeProtocol::makeData(PCUBE_HaltAll) ), port );
}
// watchdog refresh all
void Cube::watchdogAllCmd( io::CanPort* port )
{
    emitp( Cmd( cmdIdAll(),
          PCubeProtocol::makeData(PCUBE_WatchdogRefreshAll) ), port );
}

void Cube::setBaudrateAll( CubeBaudrate rate , io::CanPort* port)
{
        emitp( Cmd( cmdIdAll(),
            PCubeProtocol::makeData(PCUBE_SetBaudAll, rate) ), port );
}

void Cube::savePosAllCmd( io::CanPort* port )
{
    emitp( Cmd( cmdIdAll(),
          PCubeProtocol::makeData(PCUBE_SavePosAll) ), port );
}

void Cube::syncMotionAllCmd( io::CanPort* port )
{
    emitp( Cmd( cmdIdAll(),
          PCubeProtocol::makeData(PCUBE_SyncMotionAll) ), port );

}
// Reset, home and halt commands.

void Cube::resetCmd()
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(0)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(0)) );
}

void Cube::homeCmd()
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(0x01)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(0x01)) );
}

void Cube::haltCmd()
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(0x02)) );
    emitCmd( Cmd(cmdIdAck(), PCubeProtocol::makeData(0x02)) );
}

// Set motion commands.

// Ramp to the position val.
void Cube::moveRampCmd(float val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_MODE, 0x64)) );
}

// The time is in seconds.
void Cube::moveStepCmd(float pos, double time)
{
    // Convert to ms.
    const int ms = (int)round(time * 1000);
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_MODE, pos, ms)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_MODE, 0x64)) );
}

// Velocity in m/s.
void Cube::moveVelCmd(float val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_MODE, 0x64)) );
}

// Current in Ampere.
void Cube::moveCurCmd(float val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_MODE, 0x64)) );
}

void Cube::moveRampTicksCmd(int val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_MODE, 0x64)) );
}

void Cube::moveStepTicksCmd(int pos, double time)
{
    const int ms = (int)round(time * 1000);
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_MODE, pos, ms)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_MODE, 0x64)) );
}

// Encoder ticks per second.
void Cube::moveVelTicksCmd(int val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_MODE, 0x64)) );
}

// Current in ??
void Cube::moveCurTicksCmd(int val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_MODE, 0x64)) );
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
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_ACK, val)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_ACK)),msg ) )
        return PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        RW_WARN("No acknoledge recieved!");;
}

//Bogild
CubeExtAckData Cube::moveRampExtCmdWithState(float val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_ACK, val)) );
    io::CanPort::CanMessage msg;
	CubeExtAckData data;
	if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_ACK)),msg ) ){
		//printf("can %x %x %x %x  \n",msg.data[2],msg.data[3],msg.data[4],msg.data[5]);
		data.position = PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
		data.state = msg.data[6];
		data.dio = msg.data[7];
        return data;
	}
    else
        RW_WARN("No acknoledge recieved!");;
}

// The time is in seconds.
float Cube::moveStepExtCmd(float pos, double time)
{
    // Convert to ms.
    const int ms = (int)round(time * 1000);
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_ACK, pos, ms)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_ACK)),msg ) )
        return PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        RW_WARN("No acknoledge recieved!");;
}

// Velocity in m/s.
float Cube::moveVelExtCmd(float val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_ACK, val)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_ACK)),msg ) )
        return PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        RW_WARN("No acknoledge recieved!");;
}

//Bogild
CubeExtAckData Cube::moveVelExtCmdWithState(float val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_ACK, val)) );
    io::CanPort::CanMessage msg;
	CubeExtAckData data;
	if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_ACK)),msg ) ){
		data.position = PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
		data.state = msg.data[6];
		data.dio = msg.data[7];
        return data;
	}
    else
        RW_WARN("No acknoledge recieved!");;
}


// Current in Ampere.
float Cube::moveCurExtCmd(float val)
{
    emitCmd(Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_ACK, val)));
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_ACK)),msg ) )
        return PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        RW_WARN("No acknoledge recieved!");;
}

int Cube::moveRampTicksExtCmd(int val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_ACK, val)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_ACK)),msg ) )
        return PCubeProtocol::toInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        RW_WARN("No acknoledge recieved!");;
}

int Cube::moveStepTicksExtCmd(int pos, double time)
{
    const int ms = (int)round(time * 1000);
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_ACK, pos, ms)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_ACK)), msg ) )
        return PCubeProtocol::toInt(msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        RW_WARN("No acknoledge recieved!");;
}

// Encoder ticks per second.
int Cube::moveVelTicksExtCmd(int val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_ACK, val)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_ACK)), msg ) )
        return PCubeProtocol::toInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        RW_WARN("No acknoledge recieved!");;
}

// Current in ??
int Cube::moveCurTicksExtCmd(int val)
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_ACK, val)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_ACK)), msg) )
        return PCubeProtocol::toInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        RW_WARN("No acknoledge recieved!");;
}

// Bogild
void Cube::setConfig( unsigned int paramval ){
	emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_Config , paramval)) );
	io::CanPort::CanMessage msg;
	if( !ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_Config)), msg) )
    	RW_WARN("SET command did not function correct!!");
}

// Bogild
/*void Cube::setTargetVel( float paramval ){
	emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_TargetVel , paramval)) );
	io::CanPort::CanMessage msg;
	if( !ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_Config)), msg) )
    	RW_WARN("SET command did not function correct!!");
}

// Bogild
void Cube::setTargetAcc( float paramval ){
	emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_TargetAcc , paramval)) );
	io::CanPort::CanMessage msg;
	if( !ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetExtended, PCUBE_Config)), msg) )
    	RW_WARN("SET command did not function correct!!");
}*/


void Cube::setFloatParam( unsigned char paramid, float paramval )
{
    emitCmd( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetExtended, paramid, paramval)) );
    io::CanPort::CanMessage msg;
    if( !ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetExtended, paramid)), msg) )
    	RW_WARN("SET command did not function correct!!");
}


float Cube::getFloatParam( unsigned char paramid )
{
    emitCmd( Cmd(cmdIdGet(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::toFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        RW_WARN("No acknoledge recieved!");;
}

int Cube::getInt32Param( unsigned char paramid ){
    emitCmd( Cmd(cmdIdGet(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::toInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        RW_WARN("No acknoledge recieved!");;    
}

int Cube::getInt16Param( unsigned char paramid ){
    emitCmd( Cmd(cmdIdGet(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::toInt( 0, 0, msg.data[2], msg.data[3]);
    else
        RW_WARN("No acknoledge recieved!");;    
}

char Cube::getInt8Param( unsigned char paramid ){
    emitCmd( Cmd(cmdIdGet(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::toInt( 0, 0, 0, msg.data[2]);
    else
        RW_WARN("No acknoledge recieved!");;    
}

bool Cube::ackext(const Cmd& cmd, io::CanPort::CanMessage& msg)
{
    int sleepCnt=0;
    while( !getPort().read( msg ) ) {
        TimerUtil::sleepMs(1);
        if( sleepCnt++ > 12 ) {
            RW_WARN("Ack error: timeout!");
            return false;
        }
    } 

    if( msg.id != (unsigned int)cmd.id ){
        RW_WARN("Ack error: recieved bad id!");
        return false;
    }

    for(int i=0; i < (int)cmd.data.size(); i++){
        if( msg.data[i] != cmd.data[i] ){
            RW_WARN("Ack error: recieved bad data!");
            return false;
        }
    }

    return true;
}

bool Cube::ack(const Cmd& cmd)
{
    io::CanPort::CanMessage msg;
    int sleepCnt=0;
	while( !getPort().read( msg ) ){
        TimerUtil::sleepUs(1);
        if( sleepCnt++ > 12 ) {
            RW_WARN("Ack error: timeout!");
            return false;
        }
    }

    if( msg.id != (unsigned int)cmd.id ){
    	RW_WARN("Ack error: recieved bad id!" );
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
    getPort().write(cmd.id, cmd.data);
}
