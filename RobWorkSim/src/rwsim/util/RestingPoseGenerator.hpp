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

#ifndef RWSIM_UTIL_RESTINGPOSEGENERATOR_HPP_
#define RWSIM_UTIL_RESTINGPOSEGENERATOR_HPP_

#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <rw/kinematics/State.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwsim/simulator/Simulator.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>

#include "StateSampler.hpp"
#include "SimStateConstraint.hpp"


namespace rwsim {
namespace util {


	/**
	 * @brief finds resting poses of a dynamic scene.
	 *
	 * This generator relies on
	 *  - a state sampler for providing an initial state of the system
	 *  - a resting pose constraint for determining when the system is at rest
	 *
	 *  for each simulation step taken an update event callback a
	 *  is triggered.
	 */
	class RestingPoseGenerator {
	public:
		typedef boost::function<void(const rw::kinematics::State&)> RestingPoseCallback;
		typedef boost::function<void(const rw::kinematics::State&)> UpdateEventCallback;

		//typedef Event<StateChangedListener, StateChangedListener> StateChangedEvent;

	public:
		/**
		 *
		 * @param sim
		 * @param initState
		 * @param restConstraint
		 */
		RestingPoseGenerator(simulator::SimulatorPtr sim,
				const rw::kinematics::State& initState,
				SimStateConstraintPtr restConstraint);

		/**
		 *
		 * @param sim
		 * @param initState
		 * @param sampler
		 * @param restConstraint
		 * @return
		 */
		RestingPoseGenerator(simulator::SimulatorPtr sim,
				const rw::kinematics::State& initState,
				StateSamplerPtr sampler,
				SimStateConstraintPtr restConstraint);

		/**
		 * @brief destructor
		 */
		virtual ~RestingPoseGenerator();

		/**
		 * @brief set the sampler used for initial state
		 * @param sampler [in] state sampler
		 */
		void setInitStateSample(StateSamplerPtr sampler){
			_sampler = sampler;
		}

		/**
		 * @brief resting state contraint
		 * @param restconstraint [in] constraint
		 */
		void setRestingCriteria(SimStateConstraintPtr restconstraint){
			_restConstraint = restconstraint;
		}

		/**
		 * @brief set the callback for when a resting pose is found.
		 * @param callback [in] callback funtion
		 */
		void setResultCallback(RestingPoseCallback callback){
			_restCallback = callback;
		}

		/**
		 * @brief set the callback for when a simulation step has been carried out.
		 * @param callback [in] callback funtion
		 */
		void setUpdateEventCallback(UpdateEventCallback callback){
			_updateCallback = callback;
		}

		/**
		 * @brief start resting pose generation
		 * @param nrOfTests [in] number of resting poses to find.
		 */
		void start(int nrOfTests);

		/**
		 * @brief pause the execution
		 */
		void proceed();

		/**
		 * @brief stop the execution
		 */
		void stop();

		/**
		 * @brief the generator is finished if the specified nr of resting
		 * poses has been generated.
		 * @return true if all rest poses has been generated, false otherwise
		 */
		bool isFinished();

		/**
		 * @brief returns the number of samples that has been simulated.
		 */
		int getNrOfSamplesDone();

		/**
		 * @brief returns the number of samples that has yet to be simulated.
		 */
		int getNrOfSamplesLeft();

		/**
		 * @brief a simple status.
		 * @return
		 */
		std::string getStatusString();

	protected:
		void stepperLoop();
	private:
		simulator::ThreadSimulatorPtr _sim;

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
}
}

#endif /* SUPPORTPOSEGENERATOR_HPP_ */
