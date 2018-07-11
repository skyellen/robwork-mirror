/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "EngineTest.hpp"

#include "IntegratorGravityTest.hpp"
#include "IntegratorRotationTest.hpp"
#include "IntegratorSpringTest.hpp"

#include <rw/common/ThreadTask.hpp>
#include <rw/common/ThreadSafeVariable.hpp>

#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using rw::math::Q;
using namespace rw::trajectory;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

#define EP_NAME "rwsimlibs.test.EngineTest"
#define QUOTE(name) #name
#define ADD_EXTENSION(vector,name) \
		vector.push_back(Extension(QUOTE(name),EP_NAME,NULL,ownedPtr(new name()))); \
		vector.back().getProperties().set<std::string>("testID", QUOTE(name))

namespace {
void makeInternalExtensions(std::vector<rw::common::Extension>& internal) {
	ADD_EXTENSION(internal,IntegratorGravityTest);
	ADD_EXTENSION(internal,IntegratorRotationTest);
	ADD_EXTENSION(internal,IntegratorSpringTest);
}

class RunTask: public ThreadTask {
private:
	EngineTest* const _test;
	const std::string _engineID;
	const PropertyMap& _map;
	rw::common::Ptr<rwsim::log::SimulatorLogScope> _verbose;
	EngineTest::TestHandle::Ptr _handle;

public:
	RunTask(ThreadTask::Ptr parent, EngineTest* test, const std::string& engineID, const PropertyMap& map, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose):
		ThreadTask(parent),
		_test(test),
		_engineID(engineID),
		_map(map),
		_verbose(verbose),
		_handle(ownedPtr(new EngineTest::TestHandle()))
	{
	}

	virtual ~RunTask() {
	}

	void run() {
		try {
			_test->run(_handle, _engineID, _map, _verbose);
		} catch(const Exception& e) {
			registerFailure(e);
		}
	}

	void abort() {
		_handle->abort();
	}

	EngineTest::TestHandle::Ptr getHandle() const {
		return _handle;
	}
};
}

EngineTest::Failure::Failure(const double time, const std::string& description):
	time(time),
	description(description)
{
}

EngineTest::Result::Result(const std::string& name, const std::string& description):
	name(name),
	description(description)
{
}

void EngineTest::Result::addValue(const double time, const double val) {
	values.push_back(TimedQ(time,Q(1,val)));
}

void EngineTest::Result::addValues(const double time, const Q& vals) {
	values.push_back(TimedQ(time,vals));
}

void EngineTest::Result::checkLastValues(const double expected, const double eps) {
	const Q& val = values.back().getValue();
	for (std::size_t i = 0; i < val.size(); i++) {
		if (fabs(val[i]-expected) > eps) {
			std::stringstream str;
			str << "Value (" << val[i] << ") was not as expected (" << expected << ") - difference is " << val[i]-expected << ".";
			failures.push_back(Failure(values.back().getTime(),str.str()));
			break;
		}
	}
}

void EngineTest::Result::checkLastValuesBetween(const double expectedLow, const double expectedHigh, const double eps) {
	const Q& val = values.back().getValue();
	for (std::size_t i = 0; i < val.size(); i++) {
		if (val[i]-expectedLow < -eps || val[i]-expectedHigh > eps) {
			std::stringstream str;
			str << "Value (" << val[i] << ") was not between " << expectedLow << " and " << expectedHigh << " as expected";
			if (val[i]-expectedLow < -eps)
				str << " - lower bound violated by " << val[i]-expectedLow;
			if (val[i]-expectedHigh > eps)
				str << " - higher bound violated by " << val[i]-expectedHigh;
			str << ".";
			failures.push_back(Failure(values.back().getTime(),str.str()));
			break;
		}
	}
}

EngineTest::TestHandle::TestHandle():
	_abort(new ThreadSafeVariable<bool>(false)),
	_cb(NULL)
{
}

EngineTest::TestHandle::~TestHandle() {
	delete _abort;
}

std::string EngineTest::TestHandle::getError() const {
	return _error;
}

TimedStatePath EngineTest::TestHandle::getTimedStatePath() const {
	return _path;
}

const std::vector<EngineTest::Result>& EngineTest::TestHandle::getResults() const {
	return _results;
}

EngineTest::Result& EngineTest::TestHandle::getResult(const std::string& name) {
	BOOST_FOREACH(Result& result, _results) {
		if(result.name == name)
			return result;
	}
	RW_THROW("EngineTest::TestHandle::getResult: could not find result with that name!");
}

void EngineTest::TestHandle::setError(const std::string& error) {
	_error = error;
}

void EngineTest::TestHandle::append(const TimedState& tstate) {
	_path.push_back(tstate);
}

void EngineTest::TestHandle::append(const Result& result) {
	_results.push_back(result);
}

void EngineTest::TestHandle::addResult(const std::string& name, const std::string& description) {
	_results.push_back(Result(name,description));
}

bool EngineTest::TestHandle::isAborted() {
	return _abort->getVariable();
}

void EngineTest::TestHandle::abort() {
	_abort->setVariable(true);
}

bool EngineTest::TestHandle::success() const {
	if (_error.size() > 0)
		return false;
	BOOST_FOREACH(const Result& result, _results) {
		if (result.failures.size() > 0)
			return false;
	}
	return true;
}

void EngineTest::TestHandle::setTimeCallback(const TimeCallback cb) {
	_cb = cb;
}

void EngineTest::TestHandle::callback(const double a, const bool b, const bool c) {
	if (!_cb.empty())
		_cb(a,b,c);
}

void EngineTest::TestHandle::setDynamicWorkCell(const DynamicWorkCell::Ptr& dwc) {
	_dwc = dwc;
}

const DynamicWorkCell::Ptr& EngineTest::TestHandle::getDynamicWorkCell() const {
	return _dwc;
}

EngineTest::EngineTest()
{
}

EngineTest::~EngineTest() {
}

EngineTest::TestHandle::Ptr EngineTest::runThread(const std::string& engineID, const PropertyMap& parameters, const rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose, const ThreadTask::Ptr task) {
	if (task == NULL) {
		const TestHandle::Ptr handle = ownedPtr(new TestHandle());
		run(handle, engineID, parameters, verbose);
		return handle;
	} else {
		RunTask* const runTask = new RunTask(task, this, engineID, parameters, verbose);
		task->addSubTask(ownedPtr(runTask));
		return runTask->getHandle();
	}
}

PropertyMap::Ptr EngineTest::getDefaultParameters() const {
	return ownedPtr(new PropertyMap());
}

std::vector<PropertyMap::Ptr> EngineTest::getPredefinedParameters() const {
	std::vector<PropertyMap::Ptr> parms;
	return parms;
}

EngineTest::Factory::Factory():
	ExtensionPoint<EngineTest>(EP_NAME, "EngineTest extension point.")
	{
	if (internalExtensions().size() == 0) {
		makeInternalExtensions(internalExtensions());
	}
}

std::vector<std::string> EngineTest::Factory::getTests() {
	const EngineTest::Factory ep;
    std::vector<std::string> ids;
    BOOST_FOREACH(const Extension& ext, ep.internalExtensions()) {
    	ids.push_back(ext.getProperties().get<std::string>("testID"));
    }
    const std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(const Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("testID",ext.name) );
    }
    return ids;
}

bool EngineTest::Factory::hasTest(const std::string& test) {
	const EngineTest::Factory ep;
    BOOST_FOREACH(Extension& ext, internalExtensions()) {
        if(ext.getProperties().get<std::string>("testID") == test)
            return true;
    }
    const std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(const Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("testID",ext.name) == test)
            return true;
    }
    return false;
}

EngineTest::Ptr EngineTest::Factory::getTest(const std::string& test) {
	const EngineTest::Factory ep;
    BOOST_FOREACH(Extension& ext, internalExtensions()) {
        if(ext.getProperties().get<std::string>("testID") == test)
			return ext.getObject().cast<EngineTest>();
    }
    const std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(const Extension::Ptr& ext, exts){
		const PropertyMap& props = ext->getProperties();
		if(props.get("testID",ext->getName() ) == test){
			return ext->getObject().cast<EngineTest>();
		}
	}
	return NULL;
}

std::vector<Extension>& EngineTest::Factory::internalExtensions() {
	static std::vector<Extension> _internal;
	return _internal;
}

void EngineTest::runEngineLoop(const double dt, const TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, const rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose, const TestCallback callback, const InitCallback initialize) {
	const DynamicWorkCell::Ptr dwc = getDWC(parameters);
	if (dwc == NULL) {
		handle->setError("Could not make dynamic workcell.");
		return;
	} else {
		handle->setDynamicWorkCell(dwc);
	}
	const PhysicsEngine::Ptr engine = PhysicsEngine::Factory::makePhysicsEngine(engineID);
	if (engine == NULL) {
		handle->setError("Could not make engine.");
		return;
	}
	engine->setSimulatorLog(verbose);
	engine->load(dwc);
	const State state = dwc->getWorkcell()->getDefaultState();

	State runState = state;
	if (!initialize.empty())
		initialize(dwc,runState);
	engine->initPhysics(runState);

	EngineLoopInfo info(handle,engineID,dwc,&runState,dt);
	info.time = 0;

	if (!callback.empty())
		callback(info);

	double time = 0;
	double failTime = -1;
	bool failed = false;
	do {
		try {
			engine->step(dt,runState);
		} catch(const Exception& e) {
			failed = true;
			failTime = engine->getTime();
			handle->setError(e.what());
			break;
		} catch(...) {
			failed = true;
			failTime = engine->getTime();
			handle->setError("unknown exception!");
			break;
		}
		time = engine->getTime();

		info.time = time;
		if (!callback.empty())
			callback(info);
		failed = !handle->success();

		handle->callback(time,failed,false);
	} while (time <= getRunTime() && !handle->isAborted());

	handle->callback(time,failed,true);

	std::stringstream errString;
	if (failed) {
		errString << "failed at time " << failTime << ": " << handle->getError();
		handle->setError(errString.str());
	}

	engine->exitPhysics();
}
