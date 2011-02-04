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
	//! @addtogroup simulator @{
	/**
	 * @brief Wraps a simulator in a thread safe interface, and creates a
	 * separate thread for the simulator to run in.
	 */
	class ThreadSimulator {
	public:

	    typedef rw::common::Ptr<ThreadSimulator> Ptr;

		/**
		 * @brief constructor
		 */
		ThreadSimulator(DynamicSimulator::Ptr simulator, const rw::kinematics::State &state);

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
		DynamicSimulator::Ptr getSimulator(){
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

		bool isInError(){
			return _inError;
		}

	private:
		DynamicSimulator::Ptr _simulator;
		boost::thread *_thread;
		long _period;
		double _dt;
		rw::kinematics::State _state;
		bool _running;
		StepCallback _stepcb;
		bool _inError;
	public:
		boost::mutex _simMutex;

	};
	//! @}
}
}
#endif /* THREADSIMULATOR_HPP_ */
