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

#ifndef RWSIM_DYNAMICS_THREADSIMULATOR_HPP_
#define RWSIM_DYNAMICS_THREADSIMULATOR_HPP_

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "DynamicSimulator.hpp"


namespace rwsim {
namespace simulator {
	//! @addtogroup rwsim_simulator
	//! @{
	/**
	 * @brief Wraps a simulator in a thread safe interface, and creates a
	 * separate thread for the simulator to run in.
	 */
	class ThreadSimulator {
	public:
	    //! smart pointer type
	    typedef rw::common::Ptr<ThreadSimulator> Ptr;

		/**
		 * @brief constructor - using the default workcell state as starting state
		 */
		ThreadSimulator(DynamicSimulator::Ptr simulator);

		/**
		 * @brief constructor
		 */
		ThreadSimulator(DynamicSimulator::Ptr simulator, const rw::kinematics::State &state);

		/**
		 * @brief destructor
		 */
		virtual ~ThreadSimulator();

		/**
		 * @brief Sets the time between
		 * @param period
		 */
		//void setPeriodMs(long period);

		/**
		 * @brief This can be used to scale simulation time relative to Real World time. A scale
		 * of 1.0 makes the simulation run real time (if possible). A scale of 0.5 makes the simulation
		 * run twice as fast as real time, where a scale of 2 makes the simulation twice as slow.
		 *
		 * 1 simTime -> max( realtime * scale , simDelayInRealTime )
		 *
		 * @param scale [in]
		 */
		void setRealTimeScale(double scale){ _timescale = scale; }

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
		 * @brief same as stop but this version is non-blocking.
		 */
		void postStop(){ _postStop=true;};

		/**
		 * @brief step the simulation one \b timestep
		 */
		void step();

		/**
		 * @brief get the current state of the simuation
		 * @return
		 */
		rw::kinematics::State getState();

		/**
		 * @brief set the state of the simulator
		 * @param state [in] the new state
		 */
		void setState(const rw::kinematics::State& state);

		/**
		 * @brief reset the simulator to this state. The difference from the setState is
		 * that any changes to the non state-states such as the transform of the fixed frame,
		 * will also be updated.
		 * @param state [in] the new state
		 */
		void reset(const rw::kinematics::State& state);

		/**
		 * @brief test if this thread simulator is running
		 */
		bool isRunning(){
		    return _thread!=NULL && _running==true;
		};

		/**
		 * @brief get the current simulator time in seconds
		 */
		double getTime();

		/**
		 * @brief gets a pointer to the simulator. Make sure to stop the simulation
		 * before calling this function, otherwise an exception will be thrown.
		 * @return pointer to simulator
		 */
		DynamicSimulator::Ptr getSimulator(){
			return _simulator;
		};

		//! The callback type for a hook into the step call
		typedef boost::function<void(ThreadSimulator* sim, rw::kinematics::State&)> StepCallback;

		/**
		 * @brief if set this callback function will be called once on start and then
		 * after each step of the simulator.
		 *
		 * Set to NULL if no callback is wanted
		 */
		void setStepCallBack(StepCallback cb){
			_stepcb = cb;
		};

		/**
		 * @brief the simulator might fail because of too large penetrations. This method tests
		 * if the simulator is in an error.
		 */
		bool isInError(){
			return _inError;
		}

		/**
		 *  @brief this can be used to force the resetting of an error  state.
		 * @param inError
		 */
 		void setInError(bool inError){_inError = inError;}

	private:
		//! @brief the stepper loop
		void stepperLoop();

	private:
		DynamicSimulator::Ptr _simulator;
		boost::thread *_thread;
		//long _period;
		double _dt, _timescale;
		rw::kinematics::State _tmpState;
		bool _running;
		StepCallback _stepcb;
		bool _inError, _postStop;
	public:
		boost::mutex _simMutex, _stateMutex;
		//boost::condition _simcond;

	};
	//! @}
}
}
#endif /* THREADSIMULATOR_HPP_ */
