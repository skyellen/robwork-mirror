/*
 * ThreadSimulator.cpp
 *
 *  Created on: 25-11-2008
 *      Author: jimali
 */

#include "ThreadSimulator.hpp"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <rw/kinematics/State.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/TimerUtil.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rwsim::simulator;

namespace {

    void waitUntil(long time){
        long curr;
        do{
            boost::thread::yield();
            curr = TimerUtil::currentTimeMs();
        } while(curr<time);
    }

}


ThreadSimulator::ThreadSimulator(DynamicSimulator::Ptr simulator,
                                 const rw::kinematics::State &state):
    _simulator(simulator),
    _thread(NULL),
    _period(-1),
    _dt(0.001),
    _state(state),
    _running(false),
    _stepcb(NULL),
    _inError(false)
{
}

void ThreadSimulator::setPeriodMs(long period){
    boost::mutex::scoped_lock lock(_simMutex);
    _period = period;
}

void ThreadSimulator::setTimeStep(double dt){
    boost::mutex::scoped_lock lock(_simMutex);
    _dt = dt;
}

void ThreadSimulator::start(){
	boost::mutex::scoped_lock lock(_simMutex);
    if( _thread != NULL )
        RW_THROW("The thread is already started!");
    _running = true;
    _thread = new boost::thread(boost::bind(&ThreadSimulator::stepperLoop, this));
}

void ThreadSimulator::stop(){
   {
        boost::mutex::scoped_lock lock(_simMutex);
        _running = false;
        if( _thread==NULL ){
        	return;
        }
    }
    _thread->join();
    delete _thread;
    _thread = NULL;

}

void ThreadSimulator::step(){
    boost::mutex::scoped_lock lock(_simMutex);
	_simulator->step(_dt, _state);
}

rw::kinematics::State ThreadSimulator::getState(){
    State state;
    {
        boost::mutex::scoped_lock lock(_simMutex);
        state = _state;
    }
    return state;
}

void ThreadSimulator::setState(const rw::kinematics::State& state){
    boost::mutex::scoped_lock lock(_simMutex);
    _state = state;
    _simulator->reset(_state);
    _inError = false;
}

void ThreadSimulator::reset(const rw::kinematics::State& state){
    boost::mutex::scoped_lock lock(_simMutex);
    _state = state;
    _simulator->reset(_state);
    _inError = false;
}

double ThreadSimulator::getTime(){
    boost::mutex::scoped_lock lock(_simMutex);
    double time = _simulator->getTime();
    return time;
}

void ThreadSimulator::stepperLoop(){
    long time = TimerUtil::currentTimeMs();
    long nextTime = -1;
    bool running = true;
    while(running){


    	if(!_inError){
            boost::mutex::scoped_lock lock(_simMutex);
            nextTime = time+_period;
            running = _running;
            try {
            	_simulator->step(_dt, _state);
            } catch (...){
            	_inError = true;
            	std::cout << "ThreadSimulator Caught Nasty Error" << std::endl;
            }
        }
    	if(_inError)
    	    nextTime = time+_period;
        if(_stepcb!=NULL)
        	_stepcb(_state);
        if( nextTime>time ){
            waitUntil(nextTime);
        } else {
            boost::thread::yield();
        }
        time = nextTime;
    }
}
