/*
 * ThreadSimulator.hpp
 *
 *  Created on: 25-11-2008
 *      Author: jimali
 */

#ifndef THREADSIMULATOR_HPP_
#define THREADSIMULATOR_HPP_

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "Simulator.hpp"

/**
 * @brief Wraps a simulator in a thread safe interface, and creates a
 * separate thread for the simulator to run in.
 */
class ThreadSimulator {
public:


    /**
     * @brief constructor
     */
    ThreadSimulator(SimulatorPtr simulator, const rw::kinematics::State &state);

    /**
     * @brief destructor
     */
    virtual ~ThreadSimulator(){
        if(isRunning())
            stop();
    };

    /**
     * @brief Sets the time between
     * @param period
     */
    void setPeriodMs(long period);

    /**
     * @brief sets the timestep that will be used for the calls to
     * the step function of the simulator
     * @param dt
     */
    void setTimeStep(double dt);

    /**
     * @brief starts the simulator constraining it too the specified period
     */
    void start();

    /**
     * @brief tries to stop the simulation and blocks until the
     * thread is stopped
     */
    void stop();

    /**
     * @brief step the simulation one \b timestep
     */
    void step();

    /**
     * @brief get the current state of the simuation
     * @return
     */
    rw::kinematics::State getState();

    void setState(const rw::kinematics::State& state);

    void stepperLoop();

    bool isRunning(){ return _thread!=NULL; };

    double getTime();

    /**
     * @brief gets a pointer to the simulator. Make sure to stop the simulation
     * before calling this function, otherwise an exception will be thrown.
     * @return pointer to simulator
     */
    SimulatorPtr getSimulator(){
        return _simulator;
    };

    typedef boost::function<void(const rw::kinematics::State&)> StepCallback;

    /**
     * @brief if set this callback function will be called after each timestep
     *
     * Set to NULL if no callback is wanted
     */
    void setStepCallBack(StepCallback cb){
        _stepcb = cb;
    };

private:
	SimulatorPtr _simulator;
	boost::thread *_thread;
	long _period;
    double _dt;
    rw::kinematics::State _state;
    bool _running;
    StepCallback _stepcb;
public:
    boost::mutex _simMutex;

};

typedef rw::common::Ptr<ThreadSimulator> ThreadSimulatorPtr;

#endif /* THREADSIMULATOR_HPP_ */
