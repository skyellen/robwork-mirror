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

#include "AssemblySimulator.hpp"
#include "PhysicsEngine.hpp"

//#include <rw/common/ThreadTask.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>

#include <rwlibs/assembly/AssemblyResult.hpp>
#include <rwlibs/assembly/AssemblyControlResponse.hpp>
#include <rwlibs/assembly/AssemblyControlStrategy.hpp>
#include <rwlibs/assembly/AssemblyParameterization.hpp>
#include <rwlibs/assembly/AssemblyState.hpp>
#include <rwlibs/assembly/AssemblyTask.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/control/SerialDeviceController.hpp>
#include <rwsim/log/LogMessage.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

using namespace rw::common;
using namespace rw::invkin;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::sensor;
using namespace rw::trajectory;
using namespace rwlibs::assembly;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::simulation;
using namespace rwsim::control;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsim::sensor;
using namespace rwsim::simulator;

/*class AssemblySimulator::TaskSimulation: public ThreadTask {
public:
	TaskSimulation(AssemblySimulator* simulator, ThreadTask::Ptr parent, std::size_t taskIndex):
		ThreadTask(parent),
		_simulator(simulator),
		_taskIndex(taskIndex)
	{
	}

	void run() {
		_simulator->runSingle(_taskIndex);
	}

private:
	AssemblySimulator* _simulator;
	std::size_t _taskIndex;
};

class AssemblySimulator::TaskDispatcher: public ThreadTask {
public:
	TaskDispatcher(AssemblySimulator* simulator, ThreadTask::Ptr parent):
		ThreadTask(parent),
		_simulator(simulator)
	{
	}

	void run() {
		if (_simulator->_engineID == "ODE") {
			_simulator->runAll();
		} else {
			for (std::size_t i = 0; i < _simulator->_tasks.size(); i++) {
				ThreadTask::Ptr subtask = ownedPtr(new TaskSimulation(_simulator, this, i));
				addSubTask(subtask);
			}
		}
	}

	void done() {
		boost::mutex::scoped_lock lock(_simulator->_mutex);
		_simulator->_running = false;
	}

private:
	AssemblySimulator* _simulator;
};*/

AssemblySimulator::AssemblySimulator(rw::common::Ptr<DynamicWorkCell> dwc, const std::string &engineID, rw::common::Ptr<rwsim::contacts::ContactDetector> contactDetector, SimulatorLogScope::Ptr verbose):
	_dwc(dwc),
	_engineID(engineID),
	_contactDetector(contactDetector),
	_collisionDetector(ownedPtr(new CollisionDetector(dwc->getWorkcell(),ProximityStrategyFactory::makeDefaultCollisionStrategy()))),
	_log(verbose),
	_storeExecutionData(false),
	_postStopFinish(false),
	_postStopCancel(false),
	_running(false),
	_dt(0),
	_maxSimTime(5),
	_startInApproach(false)
{
	setDt();
	setStartInApproach();
}

AssemblySimulator::~AssemblySimulator() {
}

double AssemblySimulator::getDt() const {
	boost::mutex::scoped_lock lock(_mutex);
	return _dt;
}

void AssemblySimulator::setDt(double dt) {
	boost::mutex::scoped_lock lock(_mutex);
	if (_running)
		RW_THROW("AssemblySimulator (setDt): it is not allowed to change the timestep while the simulator is running!");
	_dt = dt;
}

void AssemblySimulator::start(rw::common::Ptr<ThreadTask> task) {
	{
		boost::mutex::scoped_lock lock(_mutex);
		_running = true;
	}
	_results.resize(_tasks.size());
	BOOST_FOREACH(AssemblyResult::Ptr &res, _results) {
		res = ownedPtr(new AssemblyResult());
	}
	if (task.isNull()) {
		runAll();
		boost::mutex::scoped_lock lock(_mutex);
		_running = false;
	} else {
		//ThreadTask::Ptr maintask = ownedPtr(new TaskDispatcher(this,task));
		//task->addSubTask(maintask);
		runAll();
	}
}

struct AssemblySimulator::SimState {
	SimState(): simulator(NULL), phase(INIT), maleFTSensor(NULL), femaleFTSensor(NULL), maleTCP(NULL), femaleTCP(NULL), saveData(true), time(0) {};
	DynamicSimulator* simulator;
	State state;
	typedef enum Phase {
		INIT,
		APPROACH,
		INSERTION,
		FINISHED,
		FAILED
	} Phase;
	Phase phase;
	BodyContactSensor::Ptr femaleContactSensor;

	// SerialDevice controlled
	SerialDeviceController::Ptr maleController;
	SerialDeviceController::Ptr femaleController;
	SerialDevice::Ptr maleDevice;
	SerialDevice::Ptr femaleDevice;

	// Body controlled
	Body::Ptr maleBodyControl;
	Body::Ptr femaleBodyControl;

	FTSensor::Ptr maleFTSensor;
	FTSensor::Ptr femaleFTSensor;
	Body::Ptr male;
	Body::Ptr female;
	Frame* maleTCP;
	Frame* femaleTCP;
	bool saveData;
	double time;

	std::vector<BodyContactSensor::Ptr> bodyContactSensors;

	Transform3D<> baseTfemale;
	Transform3D<> maleTend;
	Transform3D<> maleApproach;
	Transform3D<> femaleApproach;
	AssemblyControlStrategy::ControlState::Ptr controlState;
	Q maleTarget;
};

void AssemblySimulator::runSingle(std::size_t taskIndex, SimulatorLogScope::Ptr log) {
	State state = _dwc->getWorkcell()->getDefaultState();
	//long long time = TimerUtil::currentTimeMs();
	double simTime = 0;
	bool running = true;
	double inError = false;

	// Get local copies of shared values
	bool saveData;
	double dt;
	double maxRunTime;
	bool startInApproach;
	AssemblyTask::Ptr task;
	AssemblyResult::Ptr result;
	{
		boost::mutex::scoped_lock lock(_mutex);
		dt = _dt;
		maxRunTime = _maxSimTime;
		task = _tasks[taskIndex];
		result = _results[taskIndex];
		saveData = _storeExecutionData;
		startInApproach = _startInApproach;
	}

	SimState simState;
	simState.saveData = saveData;

	if (task->malePoseController != "") {
		simState.maleController = _dwc->findController<SerialDeviceController>(task->malePoseController);
		if (simState.maleController == NULL) {
			simState.maleBodyControl = _dwc->findBody(task->malePoseController);
		}
	}
	if (task->femalePoseController != "") {
		simState.femaleController = _dwc->findController<SerialDeviceController>(task->femalePoseController);
		if (simState.femaleController == NULL) {
			simState.femaleBodyControl = _dwc->findBody(task->femalePoseController);
		}
	}
	if ((simState.maleController == NULL && simState.maleBodyControl == NULL) && (simState.femaleController == NULL && simState.femaleBodyControl == NULL)) {
		std::cout << "Simulation could NOT be started! - there is no peg controller and no hole controller." << std::endl;
		running = false;
	}
	if (simState.maleController != NULL)
		simState.maleDevice = simState.maleController->getDynamicDevice()->getKinematicModel().cast<SerialDevice>();
	if (simState.femaleController != NULL)
		simState.femaleDevice = simState.femaleController->getDynamicDevice()->getKinematicModel().cast<SerialDevice>();

	simState.male = _dwc->findBody(task->maleID);
	simState.female = _dwc->findBody(task->femaleID);
	if (simState.male == NULL) {
		std::cout << "Simulation could NOT be started! - peg body could not be found." << std::endl;
		running = false;
	}
	if (simState.female == NULL) {
		std::cout << "Simulation could NOT be started! - hole body could not be found." << std::endl;
		running = false;
	}

	if (task->maleTCP == "")
		simState.maleTCP = simState.male->getBodyFrame();
	else
		simState.maleTCP = _dwc->getWorkcell()->findFrame(task->maleTCP);
	if (task->femaleTCP == "")
		simState.femaleTCP = simState.female->getBodyFrame();
	else
		simState.femaleTCP = _dwc->getWorkcell()->findFrame(task->femaleTCP);
	if (simState.maleTCP == NULL) {
		std::cout << "Simulation could NOT be started! - peg tcp frame could not be found." << std::endl;
		running = false;
	}
	if (simState.femaleTCP == NULL) {
		std::cout << "Simulation could NOT be started! - hole tcp frame could not be found." << std::endl;
		running = false;
	}

	if (simState.maleTCP == NULL && simState.femaleTCP == NULL) {
		std::cout << "Simulation could NOT be started! - no FTSensor found." << std::endl;
		running = false;
	}

	if (startInApproach) {
		Transform3D<> approach = task->strategy->getApproach(task->parameters);
		if (simState.maleDevice == NULL)
			simState.baseTfemale = Kinematics::worldTframe(simState.femaleTCP,state);
		else
			simState.baseTfemale = Kinematics::frameTframe(simState.maleDevice->getBase(),simState.femaleTCP,state);
		if (simState.maleDevice == NULL)
			simState.maleTend = Kinematics::frameTframe(simState.maleTCP,simState.maleBodyControl->getBodyFrame(),state);
		else
			simState.maleTend = Kinematics::frameTframe(simState.maleTCP,simState.maleDevice->getEnd(),state);
		approach = simState.baseTfemale*approach*simState.maleTend;
		simState.maleApproach = approach;

		if (simState.maleDevice != NULL) {
			// Initialize invkin solver
			InvKinSolver::Ptr solver = ownedPtr( new JacobianIKSolver(simState.maleDevice,simState.state));
			Q qPegDev = simState.maleDevice->getQ(simState.state);
			std::vector<Q> solutions = orderSolutions(solver->solve(approach, simState.state),qPegDev);
			bool success = false;
			if (solutions.size() > 0) {
				BOOST_FOREACH(Q q, solutions) {
					State state = simState.state;
					if (q.size() == 7) {
						q[5] += q[6];
						q[6] = 0;
					}
					simState.maleDevice->setQ(q,state);
					if (!_collisionDetector->inCollision(state)) {
						simState.maleTarget = q;
						success = true;
						break;
					}
				}
			}
			if (success) {
				simState.maleDevice->setQ(simState.maleTarget,simState.state);
				simState.controlState = task->strategy->createState();
				simState.phase = SimState::INSERTION;
			} else {
				std::cout << "Simulation could NOT be started! - no inverse kinematics solution found for approach." << std::endl;
				running = false;
			}
		} else {
			const RigidBody::Ptr rbody = simState.maleBodyControl.cast<RigidBody>();
			const KinematicBody::Ptr kbody = simState.maleBodyControl.cast<KinematicBody>();
		    Transform3D<> wTb = Transform3D<>::identity();
		    if (simState.maleBodyControl->getParentFrame(state) != NULL) {
		    	wTb = Kinematics::worldTframe(simState.maleBodyControl->getParentFrame(state), state);
		    }
			if (rbody != NULL) {
				rbody->getMovableFrame()->setTransform(inverse(wTb)*approach,state);
			} else if (kbody != NULL) {
				kbody->getMovableFrame()->setTransform(inverse(wTb)*approach,state);
			}
			simState.controlState = task->strategy->createState();
			simState.phase = SimState::INSERTION;
		}
	}

	simState.femaleContactSensor = ownedPtr(new BodyContactSensor("HoleContactSensor", simState.female->getBodyFrame()));
	simState.femaleContactSensor->registerIn(state);
	simState.state = state;

	PhysicsEngine::Ptr pe = PhysicsEngine::Factory::makePhysicsEngine(_engineID,_dwc);
	RW_ASSERT(pe != NULL);
	DynamicSimulator* simulator = new DynamicSimulator(_dwc,pe);
	simState.simulator = simulator;
	try {
		simulator->init(state);
	} catch(...){
		delete simulator;
		RW_THROW("could not initialize simulator!\n");
	}
	simulator->addSensor(simState.femaleContactSensor, state);

	if (task->maleFTSensor != "") {
		SimulatedFTSensor::Ptr ftsensor = _dwc->findSensor<SimulatedFTSensor>(task->maleFTSensor);
		if (ftsensor != NULL) {
			simState.maleFTSensor = ftsensor->getFTSensor(simulator).get();
			if (simState.maleController != NULL)
				simState.maleController->setFTSensor(simState.maleFTSensor.get());
		}
	}
	if (task->femaleFTSensor != "") {
		SimulatedFTSensor::Ptr ftsensor = _dwc->findSensor<SimulatedFTSensor>(task->femaleFTSensor);
		if (ftsensor != NULL) {
			simState.femaleFTSensor = ftsensor->getFTSensor(simulator);
			if (simState.femaleController != NULL)
				simState.femaleController->setFTSensor(simState.femaleFTSensor.get());
		}
	}

	BOOST_FOREACH(const std::string &name, task->bodyContactSensors) {
		SimulatedSensor::Ptr sensor = _dwc->findSensor(name);
		if (sensor != NULL) {
			BodyContactSensor::Ptr bcSensor = sensor.cast<BodyContactSensor>();
			if (bcSensor != NULL)
				simState.bodyContactSensors.push_back(bcSensor);
				simulator->addSensor(sensor, state);
		}
	}
	
	if (!_contactDetector.isNull()) {
		const bool setCD = pe->setContactDetector(_contactDetector);
		if (!setCD)
			RW_THROW("AssemblySimulator could not set ContactDetector on the used PhysicsEngine!");
	}

	if (!log.isNull())
		pe->setSimulatorLog(log);

	while(running) {
		{
			boost::mutex::scoped_lock lock(_mutex);
			if(_postStopCancel){
				running = false;
				break;
			}
		}

		std::string errorstring;
		if(!inError){
			try {
				simulator->step(dt);
				state = simulator->getState();

			} catch (std::exception& e){
				std::cout << "Error stepping" << std::endl;
				std::cout << e.what() << std::endl;
				errorstring = e.what();
				inError = 	true;
			} catch (...){
				std::cout << "Error stepping" << std::endl;
				inError = true;
			}

			// get the actual timestep taken
			double sTime = simulator->getTime();
			// calculate the time in real time that this should correspond to
			//std::cout << sTime << " " << simTime << _timescale << std::endl;

			if(sTime-simTime<0){
				// somebody reset the time...
				simTime = 0;
				//time = TimerUtil::currentTimeMs();
			}

			simTime = sTime;
		}

		if (inError) {
			running = false;
			result->error = AssemblyResult::SIMULATION_ERROR;
			result->errorMessage = errorstring;
		}

		simState.state = state;
		simState.time = simTime;
		stateMachine(simState, task, result);
		state = simState.state;

		//time = TimerUtil::currentTimeMs();

		if (simTime > maxRunTime)
			running = false;

		if (simState.phase == SimState::FAILED || simState.phase == SimState::FINISHED)
			running = false;

		//std::cout << "simTime: " << simTime << std::endl;
	}
	result->femaleTmaleEnd = Kinematics::frameTframe(simState.femaleTCP,simState.maleTCP,simState.state);
	simulator->exitPhysics();
	delete simulator;
}

void AssemblySimulator::runAll() {
	// Note: _tasks is read-only while _running is true (mutex not required)
	for (std::size_t i = 0; i < _tasks.size(); i++) {
		{
			boost::mutex::scoped_lock lock(_mutex);
			if (_postStopFinish || _postStopCancel)
				break;
		}
		SimulatorLogScope::Ptr scope = NULL;
		if (!_log.isNull()) {
			scope = ownedPtr(new SimulatorLogScope(_log.get()));
			scope->setFilename(__FILE__);
			scope->setLineBegin(__LINE__);
			std::stringstream desc;
			desc << "Assembly Task " << i << std::endl;
			scope->setDescription(desc.str());
			_log->appendChild(scope);
		}
		runSingle(i, scope);
		if (!scope.isNull()) {
			scope->setLineEnd(__LINE__);
		}
	}
}

void AssemblySimulator::stateMachine(SimState &simState, AssemblyTask::Ptr task, AssemblyResult::Ptr result) {
	// Note: for now only male object is actuated (should be expanded later for hole on peg type simulation)
	State defState = _dwc->getWorkcell()->getDefaultState();

	FTSensor* ftSensorMale;
	FTSensor* ftSensorFemale;
	if (simState.maleFTSensor != NULL)
		ftSensorMale = simState.maleFTSensor.get();
	if (simState.femaleFTSensor != NULL)
		ftSensorFemale = simState.femaleFTSensor.get();

	AssemblyState realState;
	realState.femaleTmale = Kinematics::frameTframe(simState.femaleTCP,simState.maleTCP,simState.state);
	if (task->maleFlexFrames.size() > 0) {
		for (std::size_t i = 1; i < task->maleFlexFrames.size(); i++) {
			Frame* prev = _dwc->getWorkcell()->findFrame(task->maleFlexFrames[i-1]);
			Frame* cur = _dwc->getWorkcell()->findFrame(task->maleFlexFrames[i]);
			realState.maleflexT.push_back(Kinematics::frameTframe(prev,cur,simState.state));
		}
		const Frame* flexLast = _dwc->getWorkcell()->findFrame(task->maleFlexFrames.back());
		realState.maleflexT.push_back(Kinematics::frameTframe(flexLast,simState.maleTCP,simState.state));
	}
	realState.femaleOffset = inverse(Kinematics::worldTframe(simState.femaleTCP,defState))*Kinematics::worldTframe(simState.femaleTCP,simState.state);
	if (simState.maleFTSensor != NULL)
		realState.ftSensorMale = Wrench6D<>(ftSensorMale->getForce(),ftSensorMale->getTorque());
	if (simState.femaleFTSensor != NULL)
		realState.ftSensorFemale = Wrench6D<>(ftSensorFemale->getForce(),ftSensorFemale->getTorque());
	realState.contact = hasContact(simState.femaleContactSensor,simState.male, simState.state);

	BOOST_FOREACH(const BodyContactSensor::Ptr &sensor, simState.bodyContactSensors) {
		const std::vector<Contact3D>& contacts = sensor->getContacts(simState.state);
		BOOST_FOREACH(const Contact3D& c, contacts) {
			const Transform3D<> wTsensor = Kinematics::worldTframe(sensor->getSensorModel()->getFrame(),simState.state);
			const Vector3D<> p = wTsensor*c.p;
			const Vector3D<> n = normalize(wTsensor.R()*c.n);
			const Vector3D<> force = wTsensor*c.f;
			Rotation3D<> contact = EAA<>(Vector3D<>::z(),n).toRotation3D();
			realState.contacts.push_back(Transform3D<>(p,contact));
			if (force.norm2() > realState.maxContactForce.norm2())
				realState.maxContactForce = force;
		}
	}

	AssemblyState assumedState;
	if (result->assumedState.size() > 0)
		assumedState = result->assumedState.back().getValue();
	else {
		assumedState = realState;
	}

	// Add noise to the assumed readings to simulate uncertainty!
	if (simState.maleFTSensor != NULL)
		assumedState.ftSensorMale = Wrench6D<>(ftSensorMale->getForce(),ftSensorMale->getTorque());
	if (simState.femaleFTSensor != NULL)
		assumedState.ftSensorFemale = Wrench6D<>(ftSensorFemale->getForce(),ftSensorFemale->getTorque());

	switch(simState.phase) {
	case SimState::INIT:
	{
		realState.phase = "Initialization";
		assumedState.phase = "Initialization";
		assumedState.contact = false;

		// Find approach target for Peg device
		result->approach = task->strategy->getApproach(task->parameters);
		if (simState.maleDevice == NULL)
			simState.baseTfemale = Kinematics::worldTframe(simState.femaleTCP,defState);
		else
			simState.baseTfemale = Kinematics::frameTframe(simState.maleDevice->getBase(),simState.femaleTCP,defState);
		if (simState.maleDevice == NULL)
			simState.maleTend = Kinematics::frameTframe(simState.maleTCP,simState.maleBodyControl->getBodyFrame(),defState);
		else
			simState.maleTend = Kinematics::frameTframe(simState.maleTCP,simState.maleDevice->getEnd(),defState);
		Transform3D<> approach = simState.baseTfemale*result->approach*simState.maleTend;
		simState.maleApproach = approach;

		if (simState.maleDevice != NULL) {
			// Initialize invkin solver
			InvKinSolver::Ptr solver = ownedPtr( new JacobianIKSolver(simState.maleDevice,simState.state));
			Q qPegDev = simState.maleDevice->getQ(simState.state);
			std::vector<Q> solutions = orderSolutions(solver->solve(approach, simState.state),qPegDev);
			bool success = false;
			Path<Q> qRes;
			if (solutions.size() > 0) {
				BOOST_FOREACH(Q q, solutions) {
					State state = simState.state;
					if (q.size() == 7) {
						q[5] += q[6];
						q[6] = 0;
					}
					simState.maleDevice->setQ(q,state);
					if (!_collisionDetector->inCollision(state)) {
						PlannerConstraint constraint = PlannerConstraint::make(_collisionDetector->getCollisionStrategy(),_dwc->getWorkcell(),simState.maleDevice,defState);
						QSampler::Ptr sampler = QSampler::makeUniform(simState.maleDevice);
						QMetric::Ptr metric = PlannerUtil::normalizingInfinityMetric(simState.maleDevice->getBounds());
						QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint,sampler,metric,0.005);
						if (planner->query(simState.maleDevice->getQ(simState.state),q,qRes)) {
							simState.maleTarget = q;
							success = true;
							break;
						}
					}
				}
			}
			if (success) {
				BOOST_FOREACH(const Q &q, qRes) {
					simState.maleController->movePTP(q);
				}
				simState.phase = SimState::APPROACH;
			} else {
				simState.phase = SimState::INIT;
			}
		} else {
			simState.simulator->setTarget(simState.maleBodyControl,approach,simState.state);
			simState.phase = SimState::APPROACH;
		}
	}
	break;
	case SimState::APPROACH:
	{
		realState.phase = "Approach";
		assumedState.phase = "Approach";
		assumedState.contact = false;

		// Wait until hole approach pose is reached
		Transform3D<> ct3d;
		bool controllerMoving = false;
		if (simState.maleDevice != NULL) {
			ct3d = simState.maleDevice->baseTend(simState.state);
			controllerMoving = simState.maleController->isMoving();
			simState.baseTfemale = Kinematics::frameTframe(simState.maleDevice->getBase(),simState.femaleTCP,defState);
			simState.maleTend = Kinematics::frameTframe(simState.maleTCP,simState.maleDevice->getEnd(),defState);
		} else
			ct3d = simState.maleBodyControl->getTransformW(simState.state);
		VelocityScrew6D<> dif( inverse(ct3d) * simState.maleApproach );
		bool isPoseReached = dif.linear().norm2() < 0.0005;
		//std::cout << "Approach: " << controllerMoving << " " << MetricUtil::dist2( ct3d.P(), simState.maleApproach.P() ) << " " << dif.linear().norm2() << " " << dif.angular().angle() << " " << simState.maleTarget << std::endl;
		if (isPoseReached && !controllerMoving && dif.angular().angle() < 0.01) {
			simState.controlState = task->strategy->createState();
			//_restCycles = 0;
			//_restMs = _tsim->getTime();
			simState.phase = SimState::INSERTION;
			//std::cout << "Approach: Continues to next state!" << std::endl;
		}
	}
	break;
	case SimState::INSERTION:
	{
		realState.phase = "Insertion";
		assumedState.phase = "Insertion";
		assumedState.contact = false;

		if (simState.maleDevice != NULL) {
			simState.baseTfemale = Kinematics::frameTframe(simState.maleDevice->getBase(),simState.femaleTCP,simState.state);
			simState.maleTend = Kinematics::frameTframe(simState.maleTCP,simState.maleDevice->getEnd(),simState.state);
		}

		/*Transform3D<> femaleTmale = Kinematics::frameTframe(simState.femaleTCP,simState.maleTCP,simState.state);
		bool contact = hasContact(simState.femaleContactSensor,simState.male);
		AssemblyState::Ptr real = ownedPtr(new AssemblyState());
		real->femaleTmale = femaleTmale;
		real->contact = contact;
		AssemblyState::Ptr assumed = ownedPtr(new AssemblyState());
		assumed->femaleTmale = femaleTmale;
		assumed->contact = contact;*/
		AssemblyControlResponse::Ptr response = task->strategy->update(task->parameters, &realState, &assumedState, simState.controlState, simState.state, ftSensorMale, simState.time);
		//std::cout << "Insertion: " << ftSensor->getForce() << " " << ftSensor->getTorque() << std::endl;
		//simState.phase = SimState::FINISHED;
		if (response != NULL) {
			if (response->done) {
				simState.phase = SimState::FINISHED;
				result->success = response->success;
			} else {
				if (response->type == AssemblyControlResponse::VELOCITY) {
					VelocityScrew6D<> velocity = simState.baseTfemale.R() * response->femaleTmaleVelocityTarget;
					if (simState.maleController != NULL) {
						simState.maleController->moveVelT(velocity);
					} else {
						simState.simulator->setTarget(simState.maleBodyControl,velocity);
					}
				} else {
					bool ftControl = (response->type == AssemblyControlResponse::HYBRID_FT_POS);
					VectorND<6,bool> selection = response->selection;
					float sel[6];
					if (ftControl) {
						for (std::size_t i = 0; i < 6; i++) {
							if (selection[i]) {
								ftControl = true;
								sel[i] = 0;
							} else
								sel[i] = 1;
						}
					}
					const Transform3D<> position = simState.baseTfemale * response->femaleTmaleTarget * simState.maleTend;
					if (simState.maleController != NULL) {
						if (!ftControl)
							simState.maleController->movePTP_T(position,10);
						else
							simState.maleController->moveLinFC(position,response->force_torque,sel,"",response->offset, 5);
					} else {
						if (ftControl)
							RW_THROW("AssemblySimulator (stateMachine): force/torque control not possible on free objects.");
						if (response->type == AssemblyControlResponse::POSITION)
							simState.simulator->setTarget(simState.maleBodyControl,position,simState.state);
						else if (response->type == AssemblyControlResponse::POSITION_TRAJECTORY) {
							const Trajectory<Transform3D<> >::Ptr traj = response->worldTendTrajectory;
							RW_ASSERT(traj != NULL);
							simState.simulator->setTarget(simState.maleBodyControl,traj);
						}
					}
				}
			}
		}
	}
	break;
	default:
		break;
	}
	switch(simState.phase) {
	case SimState::FINISHED:
	{
		realState.phase = "Finished";
		assumedState.phase = "Finished";
		assumedState.contact = true;
		assumedState.femaleTmale = task->femaleTmaleTarget;
	}
	break;
	case SimState::FAILED:
	{
		realState.phase = "Failed";
		assumedState.phase = "Failed";
		assumedState.contact = true;
	}
	break;
	default:
		break;
	}
	if (simState.saveData) {
		Timed<AssemblyState> trealState(simState.time,realState);
		result->realState.push_back(trealState);
		Timed<AssemblyState> tassumedState(simState.time,assumedState);
		result->assumedState.push_back(tassumedState);
	}
}

void AssemblySimulator::stopFinishCurrent() {
	boost::mutex::scoped_lock lock(_mutex);
	_postStopFinish = true;
}

void AssemblySimulator::stopCancelCurrent() {
	boost::mutex::scoped_lock lock(_mutex);
	_postStopCancel = true;
}

bool AssemblySimulator::isRunning() {
	boost::mutex::scoped_lock lock(_mutex);
	return _running;
}

void AssemblySimulator::setTasks(std::vector<AssemblyTask::Ptr> tasks) {
	boost::mutex::scoped_lock lock(_mutex);
	if (_running)
		RW_THROW("AssemblySimulator (setTasks): it is not allowed to change the tasks while the simulator is running!");
	_tasks = tasks;
}

std::vector<AssemblyResult::Ptr> AssemblySimulator::getResults() {
	boost::mutex::scoped_lock lock(_mutex);
	if (_running)
		RW_THROW("AssemblySimulator (getResults): the simulator is still running!");
	return _results;
}

void AssemblySimulator::setStoreExecutionData(bool enable) {
	boost::mutex::scoped_lock lock(_mutex);
	if (_running)
		RW_THROW("AssemblySimulator (setStoreExecutionData): the setting can not be changed while running!");
	_storeExecutionData = enable;
}

bool AssemblySimulator::storeExecutionData() {
	boost::mutex::scoped_lock lock(_mutex);
	return _storeExecutionData;
}

double AssemblySimulator::getMaxSimTime() const {
	boost::mutex::scoped_lock lock(_mutex);
	return _maxSimTime;
}

void AssemblySimulator::setMaxSimTime(double maxTime) {
	boost::mutex::scoped_lock lock(_mutex);
	if (_running)
		RW_THROW("AssemblySimulator (setMaxSimTime): the setting can not be changed while running!");
	_maxSimTime = maxTime;
}

std::vector<Q> AssemblySimulator::orderSolutions(const std::vector<Q> &solutions, const Q &curQ) {
	std::vector<Q> newSol;
	for (std::vector<Q>::const_iterator top = solutions.begin(); top != solutions.end(); top++) {
		std::vector<Q>::const_iterator best = top;
		double bestDist = (*top-curQ).normInf();
		std::vector<Q>::const_iterator it = top;
		for (it++; it != solutions.end(); it++) {
			double dist = (*it-curQ).normInf();
			if (dist < bestDist) {
				bestDist = dist;
				best = it;
			}
		}
		newSol.push_back(*best);
	}
	return newSol;
}

bool AssemblySimulator::hasContact(BodyContactSensor::Ptr sensor, Body::Ptr body, rw::kinematics::State& state)
{
	const std::vector<Body::Ptr>& bodies = sensor->getBodies(state);

	RW_ASSERT(bodies.size() == sensor->getContacts(state).size() );

	for(size_t i=0; i<bodies.size(); i++){
		if (bodies[i] == body)
			return true;
	}

	return false;
}

bool AssemblySimulator::getStartInApproach() const {
	return _startInApproach;
}

void AssemblySimulator::setStartInApproach(bool val) {
	_startInApproach = val;
}
