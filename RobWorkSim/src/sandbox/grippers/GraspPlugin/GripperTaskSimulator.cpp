#include "GripperTaskSimulator.hpp"

#include <rw/math/MetricFactory.hpp>

#define DEBUG cout


using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace rwsim::simulator;



void GripperTaskSimulator::graspFinishedCB(SimState& sstate)
{
	measureInterference(sstate, _dwc->getWorkcell()->getDefaultState());
}



double GripperTaskSimulator::measureInterference(SimState& sstate, const rw::kinematics::State& state0)
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
		//DEBUG << "INT of " << obj->getName() << ": " << objInt << endl;
		
		interference += objInt;
		interferences.push_back(objInt);
	}
	
	double wrench = sstate._target->getResult()->qualityAfterLifting(0);
	
	sstate._target->getResult()->interferenceDistances = interferenceDistances;
	sstate._target->getResult()->interferenceAngles = interferenceAngles;
	sstate._target->getResult()->interferences = interferences;
	
	if (interference > _td->getInterferenceLimit()) {
		sstate._target->getResult()->testStatus = GraspTask::Interference;
		
		DEBUG << "! INTERFERENCE: " << interference << endl;
	}
	
	if (wrench < _td->getWrenchLimit()) {
		sstate._target->getResult()->testStatus = GraspTask::ObjectSlipped;
		
		DEBUG << "! WRENCH TOO SMALL: " << wrench << endl;
	}
	
	return interference;
}
