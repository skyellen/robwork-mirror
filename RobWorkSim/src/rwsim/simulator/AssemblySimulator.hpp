/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_SIMULATOR_ASSEMBLYSIMULATOR_HPP_
#define RWSIM_SIMULATOR_ASSEMBLYSIMULATOR_HPP_

/**
 * @file AssemblyTaskSimulator.hpp
 *
 * \copydoc rwsim::simulator::AssemblyTaskSimulator
 */

#include <vector>
#include <rw/kinematics/State.hpp>
#include <boost/thread/mutex.hpp>

#include <rw/common/Ptr.hpp>

// Forward declarations
namespace rw { namespace common { class ThreadTask; }}
namespace rw { namespace math { class Q; }}
namespace rw { namespace proximity { class CollisionDetector; }}
namespace rwlibs { namespace assembly { class AssemblyTask; }}
namespace rwlibs { namespace assembly { class AssemblyResult; }}
namespace rwsim { namespace contacts { class ContactDetector; }}
namespace rwsim { namespace dynamics { class Body; }}
namespace rwsim { namespace dynamics { class DynamicWorkCell; }}
namespace rwsim { namespace log { class SimulatorLogScope; }}
namespace rwsim { namespace sensor { class BodyContactSensor; }}

namespace rwsim {
namespace simulator {

// Forward declarations
class DynamicSimulator;

//! @addtogroup rwsim_simulator

//! @{
/**
 * @brief A simulator for execution of AssemblyTasks.
 */
class AssemblySimulator {
public:
    //! @brief smart pointer type of this class
    typedef rw::common::Ptr<AssemblySimulator> Ptr;

	/**
	 * @brief Construct new simulator.
	 * @param dwc [in] the dynamic workcell.
	 * @param engineID [in] the simulator to use for dynamic simulation (for instance "ODE").
	 * @param contactDetector [in] (optional) set a contact detector that should be used by the PhysicsEngine.
	 * @param verbose [in] (optional) set a logging structure to log to.
	 */
	AssemblySimulator(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, const std::string &engineID, rw::common::Ptr<rwsim::contacts::ContactDetector> contactDetector = NULL, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose = NULL);

	//! @brief Destructor.
	virtual ~AssemblySimulator();

	/**
	 * @brief Get the size of the timestep used in simulation.
	 * @return the size of the timestep
	 */
	double getDt() const;

	/**
	 * @brief Set the size of the timestep to use in simulation.
	 * @param dt [in] the stepsize (default is 0.001 seconds).
	 */
	void setDt(double dt = 0.001);

	/**
	 * @brief Run the simulation.
	 * @param task (optional) if this simulator runs in a ThreadTask, pass a pointer to this task to let the simulator add work for parallel processing.
	 */
	void start(rw::common::Ptr<rw::common::ThreadTask> task = NULL);

	//! @brief Request stop when current tasks has finished.
	void stopFinishCurrent();

	//! @brief Request immediate stop. Current tasks are cancelled.
	void stopCancelCurrent();

	/**
	 * @brief Check if simulator is running.
	 * @return true if running.
	 */
	bool isRunning();

	/**
	 * @brief Set the tasks that should be executed in the simulator.
	 * @param tasks [in] the tasks to execute.
	 */
	void setTasks(std::vector<rw::common::Ptr<rwlibs::assembly::AssemblyTask> > tasks);

	/**
	 * @brief Retrieve the results after simulation.
	 * @return a vector of AssemblyResults.
	 */
	std::vector<rw::common::Ptr<rwlibs::assembly::AssemblyResult> > getResults();

	/**
	 * @brief Enable storing trajectory data in the AssemblyResult.
	 * @param enable [in] true if trajectory data should be stored, false otherwise.
	 */
	void setStoreExecutionData(bool enable);

	/**
	 * @brief Check if trajectory data is currently being stored.
	 * @return true if trajectory data is stored, false otherwise.
	 */
	bool storeExecutionData();

	/**
	 * @brief Get the currently set limit for the simulation time per task.
	 * @return the maximum simulated time to spend on a single task.
	 */
	double getMaxSimTime() const;

	/**
	 * @brief Set the limit for simulation time per task.
	 * @param maxTime [in] the maximum simulated time to spend on a single task.
	 */
	void setMaxSimTime(double maxTime);

	/**
	 * @brief Test whether or not the simulation is set up to start in the approach pose.
	 * @return true if simulation should start in approach pose.
	 */
	bool getStartInApproach() const;

	/**
	 * @brief Start simulation directly in the approach position as given by the strategy.
	 * @param val [in] true if simulation should start in approach pose (default is false).
	 */
	void setStartInApproach(bool val = false);

private:
	class TaskDispatcher;
	class TaskSimulation;
	struct SimState;
	void runSingle(std::size_t taskIndex, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose);
	void runAll();
	void stateMachine(SimState &state, rw::common::Ptr<rwlibs::assembly::AssemblyTask> task, rw::common::Ptr<rwlibs::assembly::AssemblyResult> result);
	static std::vector<rw::math::Q> orderSolutions(const std::vector<rw::math::Q> &solutions, const rw::math::Q &curQ);
	bool hasContact(rw::common::Ptr<rwsim::sensor::BodyContactSensor> sensor, rw::common::Ptr<rwsim::dynamics::Body> body, rw::kinematics::State& state);

private:
	const rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
	const std::string _engineID;
	const rw::common::Ptr<rwsim::contacts::ContactDetector> _contactDetector;
	const rw::common::Ptr<rw::proximity::CollisionDetector> _collisionDetector;
	const rw::common::Ptr<rwsim::log::SimulatorLogScope> _log;

	std::vector<rw::common::Ptr<rwlibs::assembly::AssemblyTask> > _tasks;
	std::vector<rw::common::Ptr<rwlibs::assembly::AssemblyResult> > _results;
	bool _storeExecutionData;
	bool _postStopFinish, _postStopCancel;
	bool _running;
	double _dt;
	double _maxSimTime;
	bool _startInApproach;
	mutable boost::mutex _mutex;
};
//! @}
} /* namespace simulator */
} /* namespace rwsim */
#endif /* RWSIM_SIMULATOR_ASSEMBLYSIMULATOR_HPP_ */
