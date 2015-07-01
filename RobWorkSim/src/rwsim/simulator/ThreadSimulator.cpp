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

    void waitUntil(long long time){
        long long curr = TimerUtil::currentTimeMs();
        while(curr<time) {
            if(1>(time-curr))
                break;
            TimerUtil::sleepMs( (int)(time-curr) );
            curr = TimerUtil::currentTimeMs();
        }
    }

}


ThreadSimulator::ThreadSimulator(rwlibs::simulation::Simulator::Ptr simulator,
                                 const rw::kinematics::State &state):
    _simulator(simulator),
    _thread(NULL),
    //_period(-1),
    _dt(0.001),
    _timescale(0.0),
    _tmpState(state),
    _running(false),
    _stepcb(NULL),
    _inError(false),
    _postStop(false)
{

}

ThreadSimulator::ThreadSimulator(rwlibs::simulation::Simulator::Ptr simulator):
    _simulator(simulator),
    _thread(NULL),
    //_period(-1),
    _dt(0.001),
    _timescale(0.0),
    _tmpState(simulator->getState()),
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

/*
void ThreadSimulator::setPeriodMs(long period){
    boost::mutex::scoped_lock lock(_simMutex);
    _period = period;
}
*/

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
    if(!_running || _postStop)
    	return;

	{
        boost::mutex::scoped_lock lock(_simMutex);
        if(!_running || _postStop)
        	return;
        _postStop = true;
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
        _simulator->step(_dt);
    }
    {
        boost::mutex::scoped_lock lock(_stateMutex);
        _tmpState = _simulator->getState();
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
    _simulator->getState() = state;
    //_simulator->reset(_state);
    _inError = false;
}

void ThreadSimulator::reset(const rw::kinematics::State& state){
    boost::mutex::scoped_lock lock(_simMutex);
    _simulator->reset(state);
    _inError = false;
}

double ThreadSimulator::getTime(){
    //boost::mutex::scoped_lock lock(_simMutex);
    double time = _simulator->getTime();
    return time;
}

void ThreadSimulator::stepperLoop(){
    long long time = TimerUtil::currentTimeMs();
    long long nextTime = time;
    double simTime = 0;
    bool running = true;
    // we call the callback once before starting
    if(_stepcb!=NULL)
         _stepcb(this, _simulator->getState());

    while(running){
		//std::cout << "!" << std::endl;
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
            //nextTime = time+_period;
            try {
            	_simulator->step(_dt);

            } catch (std::exception& e){
                Log::errorLog() << "Error stepping" << std::endl;
                Log::errorLog() << e.what() << std::endl;
                _inError = true;
            } catch (...){
				Log::errorLog() << "Error stepping" << std::endl;
            	_inError = true;
            }

            // get the actual timestep taken
            double sTime = _simulator->getTime();
            // calculate the time in real time that this should correspond to
            //std::cout << sTime << " " << simTime << _timescale << std::endl;

            if(sTime-simTime<0){
            	// somebody reset the time...
            	simTime = 0;
            	time = TimerUtil::currentTimeMs();
            	nextTime = time;
            }

            nextTime += (int)(std::min( (sTime-simTime), _dt) * _timescale * 1000);
            //std::cout << "step: " << std::min( (sTime-simTime), _dt) << std::endl;
            simTime = sTime;
        }

        {
            boost::mutex::scoped_lock lock(_stateMutex);
            _tmpState = _simulator->getState();
        }
    	if(_inError)
    	    nextTime = (int)(time+_dt*_timescale);

    	// this is necesary, since callback should not be called if user signalled stop
    	if(_postStop)
    		boost::thread::yield();

        if(_stepcb!=NULL)
        	_stepcb(this, _simulator->getState());

        time = TimerUtil::currentTimeMs();
        //std::cout << time << " --> " << nextTime << std::endl;
        if( nextTime>time ){
            waitUntil(nextTime);
        } else {
        	// if the delay is larger than one second then reset nextTime
        	if(time-nextTime>1000)
        		nextTime = time;
            boost::thread::yield();
        }

    }
}
