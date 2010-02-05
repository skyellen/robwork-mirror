/*
 * SupportPoseGenerator.hpp
 *
 *  Created on: 01-12-2008
 *      Author: jimali
 */

#ifndef RESTINGPOSEGENERATOR_HPP_
#define RESTINGPOSEGENERATOR_HPP_

#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <rw/kinematics/State.hpp>
#include <rw/trajectory/Path.hpp>
#include <simulator/Simulator.hpp>
#include <simulator/ThreadSimulator.hpp>

#include "StateSampler.hpp"
#include "SimStateConstraint.hpp"


/**
 * @brief finds resting poses of a dynamic scene and performs
 *
 */
class RestingPoseGenerator {

public:
    typedef boost::function<void(const rw::kinematics::State&)> RestingPoseCallback;
    typedef boost::function<void(const rw::kinematics::State&)> UpdateEventCallback;

    //typedef Event<StateChangedListener, StateChangedListener> StateChangedEvent;

public:
    RestingPoseGenerator(SimulatorPtr sim, const rw::kinematics::State& initState, SimStateConstraintPtr restConstraint);

    RestingPoseGenerator(SimulatorPtr sim, const rw::kinematics::State& initState, StateSamplerPtr sampler, SimStateConstraintPtr restConstraint);

    virtual ~RestingPoseGenerator();

    void setInitStateSample(StateSamplerPtr sampler){
        _sampler = sampler;
    }

    void setRestingCriteria(SimStateConstraintPtr restconstraint){
        _restConstraint = restconstraint;
    }

    void setResultCallback(RestingPoseCallback callback){
        _restCallback = callback;
    }


    void setUpdateEventCallback(UpdateEventCallback callback){
        _updateCallback = callback;
    }

    /**
     * @brief start resting pose generation
     * @param nrOfTests [in]
     */
    void start(int nrOfTests);

    void proceed();

    void stop();

protected:
    void stepperLoop();
private:
    ThreadSimulatorPtr _sim;

    StateSamplerPtr _sampler;
    SimStateConstraintPtr _restConstraint;

    bool _running, _stopRunning;

    int _nrOfTries, _nrOfTests, _maxNrOfTests;

    bool _recordStatePath, _wasInRestingState;

    rw::trajectory::TimedStatePath _statePath;

    long _updatePeriod;

    // the simulated time where resting state started
    double _timeEnteringRestingState;

    // the minimum and maximum time to simulate in
    double _minSimTime, _maxSimTime, _simStartTime;

    // the minimum time to stay in rest before valid
    double _minTimeInRest;

    boost::thread *_thread;
    boost::mutex _simMutex;

    RestingPoseCallback _restCallback;
    UpdateEventCallback _updateCallback;

    rw::kinematics::State _initState;

    std::vector<rw::kinematics::State> _initStates, _resultStates;
};

#endif /* SUPPORTPOSEGENERATOR_HPP_ */
