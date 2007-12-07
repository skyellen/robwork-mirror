//#include "include/m5apiw32.h"

#include "Cube.hpp"
#include <rwlibs/io/canbus/CanPort.hpp>

#include <rw/common/TimerUtil.hpp>

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
            std::cout << "Error writing canport!!\n";
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

    TimerUtil::SleepMs(20);

    io::CanPort::CanMessage msg;
    if( port->read( msg ) )
        return true;

    // if no ack is returned then the cube is not connected.
    return false;
}

Cube* Cube::getCubeAt( int moduleNr , io::CanPort* port )
{
    if( isCubeConnected( moduleNr , port ) ){
        return new Cube( moduleNr, port);
    }
    return NULL;
}

void Cube::resetAllAmd( io::CanPort* port )
{
    emitp( Cmd( cmdIdAll(),
          PCubeProtocol::makeData(PCUBE_HomeAll) ), port );
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
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(0)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(0)) );
}

void Cube::homeCmd()
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(0x01)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(0x01)) );
}

void Cube::haltCmd()
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(0x02)) );
    emit( Cmd(cmdIdAck(), PCubeProtocol::makeData(0x02)) );
}

// Set motion commands.

// Ramp to the position val.
void Cube::moveRampCmd(float val)
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_MODE, 0x64)) );
}

// The time is in seconds.
void Cube::moveStepCmd(float pos, double time)
{
    // Convert to ms.
    const int ms = (int)round(time * 1000);
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_MODE, pos, ms)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_MODE, 0x64)) );
}

// Velocity in m/s.
void Cube::moveVelCmd(float val)
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_MODE, 0x64)) );
}

// Current in Ampere.
void Cube::moveCurCmd(float val)
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_MODE, 0x64)) );
}

void Cube::moveRampTicksCmd(int val)
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_MODE, 0x64)) );
}

void Cube::moveStepTicksCmd(int pos, double time)
{
    const int ms = (int)round(time * 1000);
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_MODE, pos, ms)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_MODE, 0x64)) );
}

// Encoder ticks per second.
void Cube::moveVelTicksCmd(int val)
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_MODE, val)) );
    ack( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_MODE, 0x64)) );
}

// Current in ??
void Cube::moveCurTicksCmd(int val)
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_MODE, val)) );
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
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_MODE, val)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FRAMP_MODE)),msg ) )
        return PCubeProtocol::ToFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        return 0;
}

// The time is in seconds.
float Cube::moveStepExtCmd(float pos, double time)
{
    // Convert to ms.
    const int ms = (int)round(time * 1000);
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_MODE, pos, ms)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FSTEP_MODE)),msg ) )
        return PCubeProtocol::ToFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        return 0;
}

// Velocity in m/s.
float Cube::moveVelExtCmd(float val)
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_MODE, val)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FVEL_MODE)),msg ) )
        return PCubeProtocol::ToFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        return 0;
}

// Current in Ampere.
float Cube::moveCurExtCmd(float val)
{
    emit(Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_MODE, val)));
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_FCUR_MODE)),msg ) )
        return PCubeProtocol::ToFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        return 0;
}

int Cube::moveRampTicksExtCmd(int val)
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_MODE, val)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IRAMP_MODE)),msg ) )
        return PCubeProtocol::ToInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        return 0;
}

int Cube::moveStepTicksExtCmd(int pos, double time)
{
    const int ms = (int)round(time * 1000);
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_MODE, pos, ms)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ISTEP_MODE)), msg ) )
        return PCubeProtocol::ToInt(msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        return 0;
}

// Encoder ticks per second.
int Cube::moveVelTicksExtCmd(int val)
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_MODE, val)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_IVEL_MODE)), msg ) )
        return PCubeProtocol::ToInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        return 0;
}

// Current in ??
int Cube::moveCurTicksExtCmd(int val)
{
    emit( Cmd(cmdIdPut(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_MODE, val)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_SetMotion, PCUBE_ICUR_MODE)), msg) )
        return PCubeProtocol::ToInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        return 0;
}

float Cube::getFloatParam( unsigned char paramid )
{
    emit( Cmd(cmdIdGet(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::ToFloat( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        return 0;
}

int Cube::getInt32Param( unsigned char paramid ){
    emit( Cmd(cmdIdGet(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::ToInt( msg.data[2], msg.data[3], msg.data[4], msg.data[5] );
    else
        return 0;    
}

int Cube::getInt16Param( unsigned char paramid ){
    emit( Cmd(cmdIdGet(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::ToInt( 0, 0, msg.data[2], msg.data[3]);
    else
        return 0;    
}

char Cube::getInt8Param( unsigned char paramid ){
    emit( Cmd(cmdIdGet(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)) );
    io::CanPort::CanMessage msg;
    if( ackext( Cmd(cmdIdAck(), PCubeProtocol::makeData(PCUBE_GetExtended, paramid)), msg) )
        return PCubeProtocol::ToInt( 0, 0, 0, msg.data[2]);
    else
        return 0;    
}

bool Cube::ackext(const Cmd& cmd, io::CanPort::CanMessage& msg)
{
    int sleepCnt=0;
    do{
        TimerUtil::SleepMs(2);
        if( sleepCnt++ > 6 ) {
            std::cout << "Ack error: timeout!" << std::endl;
            return false;
        }
    } while( !getPort().read( msg ) );

    if( msg.id != (unsigned int)cmd.id ){
        std::cout << "Ack error: recieved bad id!" << std::endl;
        return false;
    }

    for(int i=0; i < (int)cmd.data.size(); i++){
        if( msg.data[i] != cmd.data[i] ){
            std::cout << "Ack error: recieved bad data!" <<std::endl;
            return false;
        }
    }

    return true;
}

bool Cube::ack(const Cmd& cmd)
{
    io::CanPort::CanMessage msg;
    int sleepCnt=0;
    do{
        TimerUtil::SleepMs(2);
        if( sleepCnt++ > 6 ) {
            std::cout << "Ack error: timeout!" << std::endl;
            return false;
        }
    } while( !getPort().read( msg ) );

    if( msg.id != (unsigned int)cmd.id ){
        std::cout << "Ack error: recieved bad id!" << std::endl;
        return false;
    }

    std::cout << "ACK MSG: " << msg.id ;
    for( int i=0; i<msg.length; i++)
        std::cout << "["<<(int)msg.data[i]<<"]";
    std::cout << std::endl;

    return true;
}

void Cube::emit(const Cmd& cmd)
{
    getPort().write(cmd.id, cmd.data);
}
