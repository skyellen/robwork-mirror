#include "RestingPoseGenerator.hpp"

#include "FiniteStateSampler.hpp"
#include <rw/common/Ptr.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/trajectory/Timed.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::trajectory;

RestingPoseGenerator::RestingPoseGenerator(SimulatorPtr sim, const rw::kinematics::State& initState, SimStateConstraintPtr restConstraint):
    _sim(ownedPtr(new ThreadSimulator(sim, initState))),
    _sampler(ownedPtr(new FiniteStateSampler(initState, 30000))),
    _restConstraint(restConstraint),
    _running(false),
    _stopRunning(true),
    _nrOfTries(0),
    _nrOfTests(0),
    _maxNrOfTests(0),
    _recordStatePath(false),
    _wasInRestingState(false),
    _updatePeriod(100),
    _timeEnteringRestingState(0),
    _minSimTime(0.5),
    _maxSimTime(30.0),
    _simStartTime(0),
    _minTimeInRest(0.5),
    _thread(NULL),
    _initState(initState)
{

}

RestingPoseGenerator::RestingPoseGenerator(SimulatorPtr sim, const rw::kinematics::State& initState, StateSamplerPtr sampler, SimStateConstraintPtr restConstraint):
    _sim(ownedPtr(new ThreadSimulator(sim, initState))),
    _sampler(sampler),
    _restConstraint(restConstraint),
    _running(false),
    _stopRunning(true),
    _nrOfTries(0),
    _nrOfTests(0),
    _maxNrOfTests(0),
    _recordStatePath(false),
    _wasInRestingState(false),
    _updatePeriod(100),
    _timeEnteringRestingState(0),
    _minSimTime(0.5),
    _maxSimTime(30.0),
    _simStartTime(0),
    _minTimeInRest(0.5),
    _thread(NULL),
    _initState(initState)
{

}


RestingPoseGenerator::~RestingPoseGenerator(){};

void RestingPoseGenerator::start(int nrOfTests){
    boost::mutex::scoped_lock lock(_simMutex);
    _maxNrOfTests = nrOfTests;
    _nrOfTests = 0;
    _nrOfTries = 0;
    _initStates.clear();
    _resultStates.clear();
    _statePath.clear();

    if( _thread != NULL )
        RW_THROW("The thread is already started!");
    _running = true;
    _stopRunning = false;
    _thread = new boost::thread(boost::bind(&RestingPoseGenerator::stepperLoop, this));
}

void RestingPoseGenerator::proceed(){
    boost::mutex::scoped_lock lock(_simMutex);

    if( _thread != NULL )
        RW_THROW("The thread is already started!");
    _running = true;
    _stopRunning = false;
    _thread = new boost::thread(boost::bind(&RestingPoseGenerator::stepperLoop, this));
}

void RestingPoseGenerator::stop(){
   {
        boost::mutex::scoped_lock lock(_simMutex);
        _stopRunning = true;
        if( _thread==NULL ){
            _running = false;
            return;
        }
    }
    _thread->join();
    delete _thread;
    _thread = NULL;
}

void RestingPoseGenerator::stepperLoop(){
    bool running = true;
    long time = TimerUtil::currentTimeMs();
    long nextTime = -1;

    while(running){
        // make sure not to poll the simulation too often
        time = TimerUtil::currentTimeMs();
        if( nextTime>time ){
            TimerUtil::sleepMs(_updatePeriod-(nextTime-time));
            nextTime = nextTime+_updatePeriod;
        } else {
            nextTime = time+_updatePeriod;
        }

        // now for the resting pose stuff
        bool isSimRunning = _sim->isRunning();
        if( !isSimRunning && _nrOfTests<_maxNrOfTests ){
            _sim->start();
            continue;
        }

        _sim->stop();
        State state = _sim->getState();
        double simtime = _sim->getTime();
        {
            boost::mutex::scoped_lock lock(_simMutex);
            if( _stopRunning ){
                running = false;
                continue;
            }
        }

        // if requestet add the state to the state trajectory
        if( _recordStatePath )
            _statePath.push_back( Timed<State>(simtime+_simStartTime, state) );

        _updateCallback(state);

        // don't do anything as long as we haven't simulated enough time
        if( simtime<_minSimTime ){
            _sim->start();
            continue;
        }

        bool isInRestingState = _restConstraint->isSatisfied(state, _sim->getSimulator().get());

        if( isInRestingState && !_wasInRestingState )
            _timeEnteringRestingState = simtime;
        _wasInRestingState = isInRestingState;

        // test if we are successfully done
        if( isInRestingState && (simtime-_timeEnteringRestingState)>_minTimeInRest ){
            _nrOfTries++;
            _nrOfTests++;

            // save the result
            _initStates.push_back(_initState);
            _resultStates.push_back(state);
            _restCallback(state);

            // start a new run
            _sampler->sample(_initState);
            _sim->setState(_initState);

            _simStartTime = time;
        }
        // test if the simulation ran too long
        if( simtime > _maxSimTime ){
            _sampler->sample(_initState);
            _sim->setState(_initState);
            _nrOfTries++;
        }
        _sim->start();
    }
    _running = false;
}

