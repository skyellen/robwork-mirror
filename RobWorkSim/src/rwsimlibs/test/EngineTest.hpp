/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TEST_ENGINETEST_HPP_
#define RWSIMLIBS_TEST_ENGINETEST_HPP_

/**
 * @file EngineTest.hpp
 *
 * \copydoc rwsimlibs::test::EngineTest
 */

#include <rw/common/ExtensionPoint.hpp>
#include <rw/trajectory/Path.hpp>

#include <limits>

namespace rw { namespace common { class ThreadTask; } }
namespace rw { namespace common { template <typename T> class ThreadSafeVariable; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace log { class SimulatorLogScope; } }

namespace rwsimlibs {
namespace test {
//! @addtogroup rwsimlibs_test

//! @{
/**
 * @brief Generic test type for physics engines, which provides a factory for standard tests along with an extension point for
 * user defined tests.
 */
class EngineTest {
public:
	//! @brief Smart pointer to EngineTest.
	typedef rw::common::Ptr<EngineTest> Ptr;

	//! @brief The callback type for the current simulation time
	typedef boost::function<void(double time, bool failed, bool done)> TimeCallback;

	//! @brief Format for a failure to an engine test.
	struct Failure {
		/**
		 * @brief Construct failure.
		 * @param time [in] the simulation time when the failure occurred.
		 * @param description [in] a description of the error.
		 */
		Failure(double time, const std::string& description);

		//! @brief The simulation time when the failure occurred.
		double time;
		//! @brief A description of the error.
		std::string description;
	};

	//! @brief A result of an engine test. Each test can return multiple results.
	struct Result {
		/**
		 * @brief Construct a new result.
		 * @param name [in] name of the result.
		 * @param description [in] description of the result.
		 */
		Result(const std::string& name, const std::string& description);

		/**
		 * @brief Add a value to the result.
		 * @param time [in] simulation time.
		 * @param val [in] the value to record.
		 */
		void addValue(double time, double val);

		/**
		 * @brief Add multiple values to the result.
		 * @param time [in] simulation time.
		 * @param vals [in] the values to record.
		 */
		void addValues(double time, const rw::math::Q& vals);

		/**
		 * @brief Check if the latest added value was as expected and add a failure if not.
		 * @param expected [in] the expected value.
		 * @param eps [in] (optional) the tolerance.
		 * @note For Q values with 2 or more values, these are compared to the same \b expected value.
		 */
		void checkLastValues(double expected, double eps = std::numeric_limits<double>::epsilon());

		/**
		 * @brief Check if the latest added value was in expected range, [\b expectedLow; \b expectedHigh ], and add a failure if not.
		 * @param expectedLow [in] lower bound.
		 * @param expectedHigh [in] higher bound.
		 * @param eps [in] (optional) the tolerance.
		 * @note For Q values with 2 or more values, these are compared to the same values.
		 */
		void checkLastValuesBetween(double expectedLow, double expectedHigh, double eps = std::numeric_limits<double>::epsilon());

		//! @brief Name of the result.
		std::string name;
		//! @brief Description of result.
		std::string description;
		//! @brief The timed values recorded during simulation.
		rw::trajectory::TimedQPath values;
		//! @brief A list of failures for this result.
		std::list<Failure> failures;
	};

	//! @brief Handle for a concrete test run, which makes it possible to interact with the test during simulation.
	class TestHandle {
	public:
		//! @brief Smart pointer.
		typedef rw::common::Ptr<TestHandle> Ptr;

		//! @brief Constructor.
		TestHandle();

		//! @brief Destructor.
		virtual ~TestHandle();

		/**
		 * @brief Get description of a simulation error.
		 * @note This gives only overall simulation errors. Individual results from the test can still give failures.
		 * @return empty string if there was no simulation error, or string giving the error.
		 */
		virtual std::string getError() const;

		/**
		 * @brief Get the timed state path, which will allow playback of the simulation.
		 * @return a timed state path.
		 */
		virtual rw::trajectory::TimedStatePath getTimedStatePath() const;

		/**
		 * @brief Get a vector of all results from the test run.
		 * @return a vector of Result objects.
		 */
		virtual const std::vector<Result>& getResults() const;

		/**
		 * @brief Get a reference to a specific result.
		 * @param name [in] the name of the result to find.
		 * @return the Result object with given name
		 * @throws Exception if \b name is not found.
		 */
		virtual Result& getResult(const std::string& name);

		/**
		 * @brief Set a simulation error.
		 * @param error [in] a description of the simulation error.
		 */
		virtual void setError(const std::string& error);

		/**
		 * @brief Append a timed state.
		 * @param tstate [in] the timed state.
		 */
		virtual void append(const rw::trajectory::TimedState& tstate);

		/**
		 * @brief Append with a new Result.
		 * @param result [in] the result to add.
		 */
		virtual void append(const Result& result);

		/**
		 * @brief Construct a new Result with given name.
		 * @param name [in] the name of the Result.
		 * @param description [in] the description of the result.
		 */
		virtual void addResult(const std::string& name, const std::string& description = "");

		/**
		 * @brief Check if test has been aborted.
		 * @return true if aborted.
		 */
		virtual bool isAborted();

		//! @brief Request that the simulation is aborted.
		virtual void abort();

		/**
		 * @brief Check if the test was successful.
		 * @return true if there were no simulation failures, and no failures were registered in the results.
		 */
		virtual bool success() const;

		/**
		 * @brief Set a callback to retrieve the current state of the test run.
		 * @param cb [in] the function to call.
		 */
		void setTimeCallback(TimeCallback cb);

		/**
		 * @brief Invoke the callback if it is set.
		 * @param time [in] the current simulation time.
		 * @param failed [in] whether or not simulation has failed.
		 * @param done [in] whether or not simulation has ended.
		 */
		void callback(double time, bool failed, bool done);

	private:
		std::string _error;
		rw::trajectory::TimedStatePath _path;
		std::vector<Result> _results;
		rw::common::ThreadSafeVariable<bool>* _abort;
		TimeCallback _cb;
	};

	//! @brief Construct new test.
	EngineTest();

	//! @brief Destructor.
	virtual ~EngineTest();

	/**
	 * @brief Check if engine with specific name is supported by the test.
	 * @param engineID [in] the id of the engine.
	 * @return true if supported, false otherwise.
	 */
	virtual bool isEngineSupported(const std::string& engineID) const = 0;

	/**
	 * @brief Run the test in a separate thread.
	 * @param engineID [in] the id of the engine to run test for.
	 * @param parameters [in] the parameters for the test run.
	 * @param verbose [in] record detailed internal debug information in the engine during simulation.
	 * @param task [in] the task to add simulation work to.
	 * @return a TestHandle which makes it possible to interact with the test during the test run.
	 */
	virtual TestHandle::Ptr runThread(const std::string& engineID, const rw::common::PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose, rw::common::Ptr<rw::common::ThreadTask> task);

	/**
	 * @brief Run the test.
	 * @param handle [in/out] a TestHandle where the results of the test will be added to.
	 * @param engineID [in] the id of the engine to run test for.
	 * @param parameters [in] the parameters for the test run.
	 * @param verbose [in] (optional) record detailed internal debug information in the engine during simulation.
	 */
	virtual void run(TestHandle::Ptr handle, const std::string& engineID, const rw::common::PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose = NULL) = 0;

	/**
	 * @brief Get the length of the simulation.
	 * @return the length of the simulation.
	 */
	virtual double getRunTime() const = 0;

	/**
	 * @brief Get the dynamic workcell used by the test.
	 * @param map [in] properties for test workcell.
	 * @return a smart pointer to a dynamic workcell.
	 */
	virtual rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> getDWC(const rw::common::PropertyMap& map) = 0;

	/**
	 * @brief Get the default parameters used by the test.
	 * @return the default test parameters.
	 */
	virtual rw::common::Ptr<rw::common::PropertyMap> getDefaultParameters() const;

	/**
     * @addtogroup extensionpoints
     * @extensionpoint{rwsimlibs::test::EngineTest::Factory,rwsimlibs::test::EngineTest,rwsimlibs.test.EngineTest}
     */
	/**
	 * @brief A factory for engine tests. This factory also defines an ExtensionPoint.
	 */
	class Factory: public rw::common::ExtensionPoint<EngineTest> {
	public:
		/**
		 * @brief Get the available tests.
		 * @return a vector of identifiers for tests.
		 */
		static std::vector<std::string> getTests();

		/**
		 * @brief Check if test is available.
		 * @param test [in] the name of the test.
		 * @return true if available, false otherwise.
		 */
		static bool hasTest(const std::string& test);

		/**
		 * @brief Create a new test.
		 * @param test [in] the name of the test.
		 * @return a pointer to a new EngineTest.
		 */
		static EngineTest::Ptr getTest(const std::string& test);

	private:
		Factory();
	};
	//! @}

protected:
	//! @brief Type for a function initializing the state.
	typedef boost::function<void(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell>, rw::kinematics::State&)> InitCallback;

	//! @brief The info passed in a callback function.
	struct EngineLoopInfo {
		/**
		 * @brief Construct callback info.
		 * @param handle [in] the test handle.
		 * @param engineID [in] the id for the engine being tested.
		 * @param dwc [in] the dynamic workcell.
		 * @param state [in] the current state of the simulation.
		 * @param dt [in] the time-step used.
		 */
		EngineLoopInfo(TestHandle::Ptr handle, const std::string& engineID, rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc, const rw::kinematics::State* state, double dt):
			handle(handle), engineID(engineID), dwc(dwc), state(state), dt(dt), time(0)
		{
		}

		//! @brief The test handle.
		const TestHandle::Ptr handle;
		//! @brief The current engine being tested.
		const std::string engineID;
		//! @brief The dynamic workcell.
		const rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc;
		//! @brief The current state (can not be changed).
		const rw::kinematics::State* state;
		//! @brief The time-step used.
		const double dt;
		//! @brief The current simulation time.
		double time;
	};

	//! @brief Type for a callback function.
	typedef boost::function<void(const EngineLoopInfo&)> TestCallback;

	/**
	 * @brief A standard stepping loop provided as a convenience for test implementations.
	 * @param dt [in] the time step to use.
	 * @param handle [in/out] a TestHandle where the results of the test will be added to.
	 * @param engineID [in] the id of the engine to run test for.
	 * @param parameters [in] the parameters for the test run.
	 * @param verbose [in] record detailed internal debug information in the engine during simulation.
	 * @param callback [in] (optional) call a function after each step of the simulation.
	 * @param initialize [in] (optional) call a function that sets the initial state.
	 */
	void runEngineLoop(double dt, TestHandle::Ptr handle, const std::string& engineID, const rw::common::PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose, TestCallback callback = 0, InitCallback initialize = 0);
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TEST_ENGINETEST_HPP_ */
