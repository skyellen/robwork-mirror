#include "GripperTaskSimulator.hpp"

#include <rw/math/MetricFactory.hpp>

#define DEBUG cout


using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace rwsim::simulator;



void GripperTaskSimulator::graspFinished(SimState& sstate)
{
	/* After each grasp is finished, its quality should be asessed.
	 * First, grasp has to pass interference measurement. When the sum
	 * of individual object interferences exceed limit specified in task description,
	 * grasp status is changed from Success to Interference.
	 * Then, if wrench space measurement result is below specified minimum wrench,
	 * grasp status is changed from Success to ObjectSlipped.
	 */
	if (getInterference(sstate, _td->getInitState()) > _td->getInterferenceLimit()) {
		sstate._target->getResult()->testStatus = GraspTask::Interference;
	}
	
	if (getWrench(sstate) < _td->getWrenchLimit()) {
		sstate._target->getResult()->testStatus = GraspTask::ObjectSlipped;
	}
	
	printGraspResult(sstate);
}



double GripperTaskSimulator::getInterference(SimState& sstate, const rw::kinematics::State& state0)
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
	
	return interference;
}



double GripperTaskSimulator::getWrench(SimState& sstate)
{
	return sstate._target->getResult()->qualityAfterLifting(0);
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
			DEBUG << "Grasp result " << getNrTargetsDone() << ": OTHER";
	}
}



void GripperTaskSimulator::simulationFinished(SimState& sstate)
{
}
