#include "GripperTaskSimulator.hpp"



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace rwsim::simulator;



Q poseDifference(Transform3D<> pose1, Transform3D<> pose2)
{
	double dist = MetricUtil::dist2(pose1.P(), pose2.P());
	double a = angle(pose1*Vector3D<>::z(), pose2*Vector3D<>::z());
	
	Q diff(2);
	diff[0] = dist;
	diff[1] = a;
	
	return diff;
}



GripperTaskSimulator::GripperTaskSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dwc) :
	GraspTaskSimulator(dwc, 1),
	_dwc(dwc),
	_wc(dwc->getWorkcell())
{}



void GripperTaskSimulator::graspFinishedCB()
{
	measureInterference(_dwc->getWorkcell()->getDefaultState());
}



double GripperTaskSimulator::measureInterference(const rw::kinematics::State& state0)
{
	State state1 = getSimulators()[0]->getState();
	
	BOOST_FOREACH (Object::Ptr obj, _objects) {
		Q diff = poseDifference(obj->getBase()->getTransform(state0), obj->getBase()->getTransform(state1));
		cout << obj->getName() << "- dist: " << diff[0] << " angle: " << diff[1] << endl;
	}
	
	return 1.0;
}
