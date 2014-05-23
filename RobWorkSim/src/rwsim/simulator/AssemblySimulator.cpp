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

private:
	AssemblySimulator* _simulator;
};*/

AssemblySimulator::AssemblySimulator(rw::common::Ptr<DynamicWorkCell> dwc, const std::string &engineID, rw::common::Ptr<rwsim::contacts::ContactDetector> contactDetector):
	_dwc(dwc),
	_engineID(engineID),
	_contactDetector(contactDetector),
	_collisionDetector(ownedPtr(new CollisionDetector(dwc->getWorkcell(),ProximityStrategyFactory::makeDefaultCollisionStrategy()))),
	_storeExecutionData(false),
	_postStopFinish(false),
	_postStopCancel(false),
	_dt(0.001)
{
}

AssemblySimulator::~AssemblySimulator() {
}

void AssemblySimulator::start(rw::common::Ptr<ThreadTask> task) {
	_results.resize(_tasks.size());
	BOOST_FOREACH(AssemblyResult::Ptr &res, _results) {
		res = ownedPtr(new AssemblyResult());
	}
	if (task == NULL) {
		runAll();
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

	FTSensor* maleFTSensor;
	FTSensor* femaleFTSensor;
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

void AssemblySimulator::runSingle(std::size_t taskIndex) {
	State state = _dwc->getWorkcell()->getDefaultState();
	//long long time = TimerUtil::currentTimeMs();
	double simTime = 0;
	bool running = true;
	double inError = false;

	// Get local copies of shared values
	bool saveData;
	double dt;
	AssemblyTask::Ptr task;
	AssemblyResult::Ptr result;
	{
		boost::mutex::scoped_lock lock(_mutex);
		dt = _dt;
		task = _tasks[taskIndex];
		result = _results[taskIndex];
		saveData = _storeExecutionData;
	}

	SimState simState;
	simState.saveData = saveData;

	simState.state = state;
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

	if (task->maleFTSensor != "") {
		SimulatedSensor::Ptr sensor = _dwc->findSensor(task->maleFTSensor);
		if (sensor != NULL) {
			SimulatedFTSensor::Ptr ftSensor = sensor.cast<SimulatedFTSensor>();
			simState.maleFTSensor = ftSensor->getSensor().cast<FTSensor>().get();
			if (simState.maleController != NULL)
				simState.maleController->setFTSensor(simState.maleFTSensor);
		}
	}
	if (task->femaleFTSensor != "") {
		SimulatedSensor::Ptr sensor = _dwc->findSensor(task->femaleFTSensor);
		if (sensor != NULL) {
			SimulatedFTSensor::Ptr ftSensor = sensor.cast<SimulatedFTSensor>();
			simState.femaleFTSensor = ftSensor->getSensor().cast<FTSensor>().get();
			if (simState.femaleController != NULL)
				simState.femaleController->setFTSensor(simState.femaleFTSensor);
		}
	}
	if (simState.maleTCP == NULL && simState.femaleTCP == NULL) {
		std::cout << "Simulation could NOT be started! - no FTSensor found." << std::endl;
		running = false;
	}

	PhysicsEngine::Ptr pe = PhysicsEngine::Factory::makePhysicsEngine(_engineID,_dwc);
	RW_ASSERT(pe != NULL);
	DynamicSimulator* simulator = new DynamicSimulator(_dwc,pe);
	simState.simulator = simulator;
	simState.femaleContactSensor = ownedPtr(new BodyContactSensor("HoleContactSensor", simState.femaleTCP));
	try {
		simulator->init(state);
	} catch(...){
		delete simulator;
		RW_THROW("could not initialize simulator!\n");
	}
	simulator->addSensor(simState.femaleContactSensor, state);

	BOOST_FOREACH(const std::string &name, task->bodyContactSensors) {
		SimulatedSensor::Ptr sensor = _dwc->findSensor(name);
		if (sensor != NULL) {
			BodyContactSensor::Ptr bcSensor = sensor.cast<BodyContactSensor>();
			if (bcSensor != NULL)
				simState.bodyContactSensors.push_back(bcSensor);
				simulator->addSensor(sensor, state);
		}
	}
	
	{
		bool setCD = pe->setContactDetector(_contactDetector);
		if (_contactDetector != NULL && !setCD)
			RW_THROW("AssemblySimulator could not set ContactDetector on the used PhysicsEngine!");
	}

	while(running){
		{
			boost::mutex::scoped_lock lock(_mutex);
			if(_postStopCancel){
				running = false;
				break;
			}
		}

		if(!inError){
			try {
				simulator->step(dt, state);

			} catch (std::exception& e){
				std::cout << "Error stepping" << std::endl;
				std::cout << e.what() << std::endl;
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

		if (inError)
			running = false;

		simState.state = state;
		simState.time = simTime;
		stateMachine(simState, task, result);
		state = simState.state;

		//time = TimerUtil::currentTimeMs();

		if (simTime > 5)
			running = false;

		std::cout << "simTime: " << simTime << std::endl;
	}
	result->femaleTmaleEnd = Kinematics::frameTframe(simState.femaleTCP,simState.maleTCP,simState.state);
	simulator->exitPhysics();
	delete simulator;
}

void AssemblySimulator::runAll() {
	for (std::size_t i = 0; i < _tasks.size(); i++) {
		if (_postStopFinish || _postStopCancel)
			return;
		runSingle(i);
	}
}

void AssemblySimulator::stateMachine(SimState &simState, AssemblyTask::Ptr task, AssemblyResult::Ptr result) {
	// Note: for now only male object is actuated (should be expanded later for hole on peg type simulation)
	State defState = _dwc->getWorkcell()->getDefaultState();

	FTSensor* ftSensorMale;
	FTSensor* ftSensorFemale;
	if (simState.maleFTSensor != NULL)
		ftSensorMale = simState.maleFTSensor;
	if (simState.femaleFTSensor != NULL)
		ftSensorFemale = simState.femaleFTSensor;

	AssemblyState realState;
	AssemblyState assumedState;
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
		std::cout << "found female sensor!" << std::endl;
	if (simState.femaleFTSensor != NULL)
		realState.ftSensorFemale = Wrench6D<>(ftSensorFemale->getForce(),ftSensorFemale->getTorque());
	realState.contact = hasContact(simState.femaleContactSensor,simState.male);

	// Add noise to the assumed readings to simulate uncertainty!
	if (simState.maleFTSensor != NULL)
		assumedState.ftSensorMale = Wrench6D<>(ftSensorMale->getForce(),ftSensorMale->getTorque());
	if (simState.femaleFTSensor != NULL)
		assumedState.ftSensorFemale = Wrench6D<>(ftSensorFemale->getForce(),ftSensorFemale->getTorque());

	BOOST_FOREACH(const BodyContactSensor::Ptr &sensor, simState.bodyContactSensors) {
		const std::vector<Contact3D>& contacts = sensor->getContacts();
		BOOST_FOREACH(const Contact3D& c, contacts) {
			const Transform3D<> wTsensor = Kinematics::worldTframe(sensor->getSensorFrame(),simState.state);
			const Vector3D<> p = wTsensor*c.p;
			const Vector3D<> n = normalize(wTsensor.R()*c.n);
			Rotation3D<> contact = EAA<>(Vector3D<>::z(),n).toRotation3D();
			realState.contacts.push_back(Transform3D<>(p,contact));
		}
	}

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
		} else
			ct3d = simState.maleBodyControl->getTransformW(simState.state);
		bool isPoseReached = MetricUtil::dist2( ct3d.P(), simState.maleApproach.P() )<0.0005;
		VelocityScrew6D<> dif( inverse(ct3d) * simState.maleApproach );
		std::cout << "Approach: " << controllerMoving << " " << MetricUtil::dist2( ct3d.P(), simState.maleApproach.P() ) << " " << dif.linear().norm2() << " " << dif.angular().angle() << " " << simState.maleTarget << std::endl;
		if (isPoseReached && !controllerMoving && dif.angular().angle() < 0.01) {
			simState.controlState = task->strategy->createState();
			//_restCycles = 0;
			//_restMs = _tsim->getTime();
			simState.phase = SimState::INSERTION;
			std::cout << "Approach: Continues to next state!" << std::endl;
		}
	}
	break;
	case SimState::INSERTION:
	{
		realState.phase = "Insertion";
		assumedState.phase = "Insertion";
		assumedState.contact = false;

		Transform3D<> femaleTmale = Kinematics::frameTframe(simState.femaleTCP,simState.maleTCP,simState.state);
		bool contact = hasContact(simState.femaleContactSensor,simState.male);
		AssemblyState::Ptr real = ownedPtr(new AssemblyState());
		real->femaleTmale = femaleTmale;
		real->contact = contact;
		AssemblyState::Ptr assumed = ownedPtr(new AssemblyState());
		assumed->femaleTmale = femaleTmale;
		assumed->contact = contact;
		AssemblyControlResponse::Ptr response = task->strategy->update(task->parameters, real, assumed, simState.controlState, simState.state, ftSensorMale);
		//std::cout << "Insertion: " << ftSensor->getForce() << " " << ftSensor->getTorque() << std::endl;
		//simState.phase = SimState::FINISHED;
		if (response != NULL) {
			if (response->done)
				simState.phase = SimState::FINISHED;
			else {
				if (response->type == AssemblyControlResponse::VELOCITY) {
					VelocityScrew6D<> velocity = simState.baseTfemale.R() * response->femaleTmaleVelocityTarget;
					if (simState.maleController != NULL) {
						simState.maleController->moveVelT(velocity);
					} else {
						simState.simulator->setTarget(simState.maleBodyControl,velocity,simState.state);
					}
				} else {
					bool ftControl = (response->type == AssemblyControlResponse::HYBRID_FT_POS);
					VectorND<6,bool> selection = response->selection;
					float sel[6];
					for (std::size_t i = 0; i < 6; i++) {
						if (selection[i]) {
							ftControl = true;
							sel[i] = 0;
						} else
							sel[i] = 1;
					}
					Transform3D<> position = simState.baseTfemale * response->femaleTmaleTarget * simState.maleTend;
					if (simState.maleController != NULL) {
						if (!ftControl)
							simState.maleController->movePTP_T(position,5);
						else
							simState.maleController->moveLinFC(position,response->force_torque,sel,"",response->offset, 5);
					} else {
						if (!ftControl)
							simState.simulator->setTarget(simState.maleBodyControl,position,simState.state);
						else
							simState.simulator->setTarget(simState.maleBodyControl,position,simState.state);
					}
				}
			}
		}
	}
	break;
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
	}
	if (simState.saveData) {
		Timed<AssemblyState> trealState(simState.time,realState);
		result->realState.push_back(trealState);
		Timed<AssemblyState> tassumedState(simState.time,assumedState);
		result->assumedState.push_back(tassumedState);
	}
}

void AssemblySimulator::stopFinishCurrent() {
	_postStopFinish = true;
}

void AssemblySimulator::stopCancelCurrent() {
	_postStopCancel = true;
}

void AssemblySimulator::setTasks(std::vector<AssemblyTask::Ptr> tasks) {
	_tasks = tasks;
}

std::vector<AssemblyResult::Ptr> AssemblySimulator::getResults() {
	return _results;
}

void AssemblySimulator::setStoreExecutionData(bool enable) {
	_storeExecutionData = enable;
}

bool AssemblySimulator::storeExecutionData() {
	return _storeExecutionData;
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

bool AssemblySimulator::hasContact(BodyContactSensor::Ptr sensor, Body::Ptr body)
{
	const std::vector<rw::sensor::Contact3D>& contacts = sensor->getContacts();
	const std::vector<Body::Ptr>& bodies = sensor->getBodies();

	RW_ASSERT(bodies.size() == contacts.size() );

	for(size_t i=0; i<bodies.size(); i++){
		if (bodies[i] == body)
			return true;
	}

	return false;
}
