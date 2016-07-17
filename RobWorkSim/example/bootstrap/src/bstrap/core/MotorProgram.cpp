#include "MotorProgram.hpp"

#include <rw/common/TimerUtil.hpp>

using namespace rw::common;

MotorProgram::MotorProgram(const std::string& name, QWidget *parent) :
            QThread(parent), _name(name), _stopped(true)
{
}

void MotorProgram::execute(rw::common::Ptr<rw::common::PropertyMap> parameters, const BrainState& bstate){
    setParameters(parameters, bstate);
    start();
}


void MotorProgram::run() {
     _stop = false;
     _stopped = false;

     while( _stop==false ){
         executionloop();
         TimerUtil::sleepMs(0.1);
     }
     _stopped = true;
 }


void MotorProgram::stop(){ _stop=true; }

bool MotorProgram::isStopped(){ return _stopped; }
