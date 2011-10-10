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
        long curr = TimerUtil::currentTimeMs();
        while(curr<time) {
            if(1>(time-curr))
                break;
            TimerUtil::sleepMs( time-curr );
            curr = TimerUtil::currentTimeMs();
        }
    }

}


ThreadSimulator::ThreadSimulator(DynamicSimulator::Ptr simulator,
                                 const rw::kinematics::State &state):
    _simulator(simulator),
    _thread(NULL),
    _period(-1),
    _dt(0.001),
    _state(state),
    _tmpState(state),
    _running(false),
    _stepcb(NULL),
    _inError(false),
    _postStop(false)
{
}

ThreadSimulator::~ThreadSimulator(){
    if(isRunning())
        stop();
};


void ThreadSimulator::setPeriodMs(long period){
    boost::mutex::scoped_lock lock(_simMutex);
    _period = period;
}

void ThreadSimulator::setTimeStep(double dt){
    boost::mutex::scoped_lock lock(_simMutex);
    _dt = dt;
}

void ThreadSimulator::start(){
    stop();
    {
        boost::mutex::scoped_lock lock(_simMutex);
        _running = true;
        _thread = new boost::thread(boost::bind(&ThreadSimulator::stepperLoop, this));
    }
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
    {
        boost::mutex::scoped_lock lock(_simMutex);
        _simulator->step(_dt, _state);
    }
    {
        boost::mutex::scoped_lock lock(_stateMutex);
        _tmpState = _state;
    }
}

rw::kinematics::State ThreadSimulator::getState(){
    State state;
    {
        boost::mutex::scoped_lock lock(_stateMutex);
        state = _tmpState;
    }
    return state;
}

void ThreadSimulator::setState(const rw::kinematics::State& state){
    boost::mutex::scoped_lock lock(_simMutex);
    _state = state;
    //_simulator->reset(_state);
    _inError = false;
}

void ThreadSimulator::reset(const rw::kinematics::State& state){
    boost::mutex::scoped_lock lock(_simMutex);
    _state = state;
    _simulator->reset(_state);
    _inError = false;
}

double ThreadSimulator::getTime(){
    //boost::mutex::scoped_lock lock(_simMutex);
    double time = _simulator->getTime();
    return time;
}

void ThreadSimulator::stepperLoop(){
    long time = TimerUtil::currentTimeMs();
    long nextTime = -1;
    bool running = true;
    // we call the callback once before starting
    if(_stepcb!=NULL)
         _stepcb(this, _state);

    while(running){
        {
            boost::mutex::scoped_lock lock(_simMutex);
            if(_postStop){
                _postStop = false;
                running = false;
                _running = false;
                break;
            }
            running = _running;
        }

    	if(!_inError){
            boost::mutex::scoped_lock lock(_simMutex);
            nextTime = time+_period;
            try {
            	_simulator->step(_dt, _state);
            } catch (...){
            	_inError = true;
            	//std::cout << "ThreadSimulator Caught Nasty Error" << std::endl;
            }
        }
        {
            boost::mutex::scoped_lock lock(_stateMutex);
            _tmpState = _state;
        }
    	if(_inError)
    	    nextTime = time+_period;
        if(_stepcb!=NULL)
        	_stepcb(this, _state);

        if( nextTime>time ){
            waitUntil(nextTime);
        } else {
            boost::thread::yield();
        }
        time = nextTime;
    }
    _thread = NULL;
}
