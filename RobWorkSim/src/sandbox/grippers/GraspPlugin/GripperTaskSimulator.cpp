#include "GripperTaskSimulator.hpp"

#include <rw/math/MetricFactory.hpp>
#include "TaskGenerator.hpp"

#define DEBUG cout


using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace rwsim::simulator;
using namespace rw::common;



GripperTaskSimulator::GripperTaskSimulator(rw::models::Gripper::Ptr gripper, rwlibs::task::GraspTask::Ptr tasks,
		rwlibs::task::GraspTask::Ptr samples, TaskDescription::Ptr td) :
	GraspTaskSimulator(td->getDynamicWorkCell(), 1),
	_gripper(gripper),
	_samples(samples),
	_td(td)
{
	load(tasks);
}



void GripperTaskSimulator::graspFinished(SimState& sstate)
{
	/* After each grasp is finished, its quality should be asessed.
	 * First, grasp has to pass interference measurement. When the sum
	 * of individual object interferences exceed limit specified in task description,
	 * grasp status is changed from Success to Interference.
	 * Then, if wrench space measurement result is below specified minimum wrench,
	 * grasp status is changed from Success to ObjectSlipped.
	 */
	if (!_td) {
		RW_THROW("NULL task description!");
	}
	
	if (calculateInterference(sstate, _td->getInitState()) > _td->getInterferenceLimit()) {
		sstate._target->getResult()->testStatus = GraspTask::Interference;
	}
	
	if (calculateWrench(sstate) < _td->getWrenchLimit()) {
		sstate._target->getResult()->testStatus = GraspTask::ObjectSlipped;
	}
	
	printGraspResult(sstate);
}



double GripperTaskSimulator::calculateInterference(SimState& sstate, const rw::kinematics::State& state0)
{
	State state1 = getSimulators()[0]->getState();
	
	double interference = 0.0;
	vector<double> interferenceDistances;
	vector<double> interferenceAngles;
	vector<double> interferences;
	
	BOOST_FOREACH (Object::Ptr obj, _td->getInterferenceObjects()) {
		
		Transform3D<> Tbefore = obj->getBase()->getTransform(state0);
		Transform3D<> Tafter = obj->getBase()->getTransform(state1);
		
		double d = MetricUtil::dist2(Tafter.P(), Tbefore.P());
		interferenceDistances.push_back(d);
		
		double a = angle(Tbefore*Vector3D<>::z(), Tafter*Vector3D<>::z());
		interferenceAngles.push_back(a);
		
		Metric<Transform3D<> >::Ptr metric = MetricFactory::makeTransform3DMetric<double>(1.0, 1.0);
		double objInt = metric->distance(Tbefore, Tafter);
		
		interference += objInt;
		interferences.push_back(objInt);
	}
	
	sstate._target->getResult()->interferenceDistances = interferenceDistances;
	sstate._target->getResult()->interferenceAngles = interferenceAngles;
	sstate._target->getResult()->interferences = interferences;
	sstate._target->getResult()->interference = interference;
	
	return interference;
}



double GripperTaskSimulator::calculateWrench(SimState& sstate) const
{
	return sstate._target->getResult()->qualityAfterLifting(1); // use average wrench from origin
}



double GripperTaskSimulator::calculateCoverage()
{
	if (!_gtask || !_samples) {
		RW_WARN("NULL tasks or samples");
		return 0.0;
	}
	
	double coverage = 0.0;

	Q covDist = _td->getCoverageDistance();
	double R = 2.0 * sin(0.25 * covDist(1));
	Q diff(7, covDist(0), covDist(0), covDist(0), R, R, R, covDist(2));
	//cout << "!!!" << diff << endl;
	//diff = Q(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 45*Deg2Rad);

	int okTargets = TaskGenerator::countTasks(TaskGenerator::filterTasks(_gtask, diff), GraspTask::Success);
	int allTargets = TaskGenerator::countTasks(TaskGenerator::filterTasks(_samples, diff), GraspTask::Success);
	
	DEBUG << "N of tasks: " << getNrTargets() << " / N of all samples: " << _samples->getSubTasks()[0].getTargets().size() << endl;
	DEBUG << "Filtered grasps: " << okTargets << " / Parallel samples: " << allTargets << endl;
	coverage = 1.0 * okTargets / allTargets;
	
	return coverage;
}



rw::math::Q GripperTaskSimulator::calculateWrenchMeasurement() const
{
	Q wrench(2, 0, 0);
	
	int success = 0;
	typedef std::pair<class GraspSubTask*, class GraspTarget*> TaskTarget;
	//DEBUG << "WRENCHES!" << endl;
	BOOST_FOREACH (TaskTarget p, _gtask->getAllTargets()) {
		//DEBUG << "??? " << p.second->getResult()->testStatus << endl;
		if (p.second->getResult()->testStatus == GraspTask::Success) {
			success++;
			
			Q result = p.second->getResult()->qualityAfterLifting;
			//DEBUG << result << endl;
			
			wrench(0) += result(0);
			wrench(1) += result(1);
		}
	}
	
	return wrench / success;
}



double GripperTaskSimulator::calculateShape()
{
	return 0.0;
}



double GripperTaskSimulator::calculateQuality()
{
	return 0.0;
}



void GripperTaskSimulator::printGraspResult(SimState& sstate)
{
	int status = sstate._target->getResult()->testStatus;
	
	switch (status) {
		case GraspTask::Success:
			DEBUG << "Grasp result " << getNrTargetsDone() << ": success" << endl;
			break;
			
		case GraspTask::Interference:
			DEBUG << "Grasp result " << getNrTargetsDone() << ": INTERFERENCE" << endl;
			break;
			
		case GraspTask::ObjectSlipped:
			DEBUG << "Grasp result " << getNrTargetsDone() << ": SLIPPED or INSUFFICIENT WRENCH" << endl;
			break;
			
		case GraspTask::TimeOut:
			DEBUG << "Grasp result " << getNrTargetsDone() << ": TIMEOUT" << endl;
			break;
			
		default:
			DEBUG << "Grasp result " << getNrTargetsDone() << ": OTHER(" << status << ")" << endl;
	}
	
	DEBUG << " I: " << sstate._target->getResult()->interference;
	DEBUG << " W: " << sstate._target->getResult()->qualityAfterLifting << endl;
}



void GripperTaskSimulator::simulationFinished(SimState& sstate)
{
	evaluateGripper();
}



void GripperTaskSimulator::evaluateGripper()
{
	/* This function performs gripper quality evaluation.
	 * This includes:
	 * - success ratio calculation
	 * - wrench space measurement
	 * - coverage calculation
	 */
	TaskDescription::Qualities& b = _td->getBaseline();
	TaskDescription::Qualities& w = _td->getWeights();
	 
	DEBUG << "Evaluating..." << endl;
	
	int successes = TaskGenerator::countTasks(_gtask, GraspTask::Success);
	int samples = _samples->getSubTasks()[0].getTargets().size();
	
	//double shape = calculateShape() / b.shape;
	Q wrenchMeasurement = calculateWrenchMeasurement();
	double wrench = wrenchMeasurement(0) / b.wrench;
	double coverage = calculateCoverage() / b.coverage;
	double successRatio = (1.0 * successes / getNrTargets()) / b.success;
	
	double sumWeights = w.shape + w.coverage + w.success + w.wrench;
	double quality = (
		//w.shape * shape +
		w.coverage * coverage +
		w.success * successRatio +
		w.wrench * wrench
		) / sumWeights;
	
	// save data to gripper result
	GripperQuality::Ptr q = _gripper->getQuality();
	q->nOfExperiments = getNrTargets();
	q->nOfSuccesses = successes;
	q->nOfSamples = samples;
	//q->shape = shape;
	q->coverage = coverage;
	q->success = successRatio;
	q->wrench = wrench;
	q->quality = quality;
	
	DEBUG << *q << endl;
}
