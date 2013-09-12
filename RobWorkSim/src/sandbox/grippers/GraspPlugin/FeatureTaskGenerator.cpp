#include "FeatureTaskGenerator.hpp"

#include <rw/geometry/TriMeshSurfaceSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>



using namespace std;

USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;




FeatureTaskGenerator::FeatureTaskGenerator(rwsim::dynamics::DynamicWorkCell::Ptr dwc, const std::string& objectID, const std::string& gripperID) :
	TaskGenerator(dwc, objectID, gripperID)
{
}



std::vector<FeatureTaskGenerator::Feature> FeatureTaskGenerator::_createFeatures(std::vector<Point>& points, int nPoints)
{
	Geometry::Ptr geo = _object->getGeometry()[0];
	
	TriMeshSurfaceSampler sampler(geo);
	sampler.setRandomPositionEnabled(false);
	sampler.setRandomRotationEnabled(false);
	
	cout << "Sampling points... ";
	for (int i = 0; i < nPoints; ++i) {
		Transform3D<> t = sampler.sample();
		
		points.push_back(make_pair(t.P(), t.R() * Vector3D<>::z()));
	}
	cout << "Done." << endl;
	
	cout << "Creating features... ";
	vector<Feature> features;
	
	for (int i = 0; i < nPoints; ++i) {
		Point& p1 = points[i];
		for (int j = 0; j < nPoints; ++j) {
			Point& p2 = points[j];
			bool closeAngle = angle(-p1.second, p2.second) < 50*Deg2Rad;
            double dist = MetricUtil::dist2(p1.first, p2.first);
            
            if (closeAngle && (dist > _openQ[0]+_jawdist) && (dist < _closeQ[0]*2.0+_jawdist)) {
                features.push_back(make_pair(i, j));
            }
		}
	}
	cout << "Done." << endl;
	
	return features;
}



rwlibs::task::GraspTask::Ptr FeatureTaskGenerator::generateTask(int nTargets, rw::proximity::CollisionDetector::Ptr cdetect)
{
	State state = _wc->getDefaultState();
	Transform3D<> wTobj = Kinematics::worldTframe(_object->getBase(), state);
	
	// setup task
	GraspTask::Ptr gtask = new GraspTask;
	gtask->getSubTasks().resize(1);
	GraspSubTask& stask = gtask->getSubTasks()[0];
	gtask->setGripperID(_gripperID);
    stask.offset = wTobj;
    stask.approach = Transform3D<>(Vector3D<>(0, 0, 0.1));
    stask.retract = Transform3D<>(Vector3D<>(0, 0, 0.1));
    stask.openQ = _openQ;
    stask.closeQ = _closeQ;
    gtask->setTCPID(_gripperTCP->getName());
    gtask->setGraspControllerID("graspController");
    
    // configure collision detector
	//CollisionStrategy::Ptr cstrategy = ProximityStrategyFactory::makeDefaultCollisionStrategy();
    //CollisionDetector cdetect(_wc, cstrategy);
	
    //ProximityModel::Ptr object = cstrategy->createModel();
    //cstrategy->addGeometry(object.get(), _object->getGeometry()[0]);

    // generate targets
    if (_features.size() == 0) _features = _createFeatures(_points);
    
    for (int successes = 0; successes < nTargets;) {
		// first choose a feature
		int i = Math::ranI(0, _features.size()-1);
        Feature sfeat = _features[i];
        
        Vector3D<> p1 = _points[sfeat.first].first;
        Vector3D<> p2 = _points[sfeat.second].first;
        Vector3D<> tcp_p = (p2-p1)/2.0 + p1;
        
        // generate orientation, xaxis determine gripper closing direction, zaxis the approach
        Vector3D<> xaxis = normalize(p2-p1);
        Vector3D<> yaxis = normalize(cross(Vector3D<>(Math::Math::ranNormalDist(0, 1), Math::Math::ranNormalDist(0, 1), Math::Math::ranNormalDist(0, 1)), xaxis));
        Vector3D<> zaxis = normalize(cross(xaxis, yaxis));
        Transform3D<> target(tcp_p, Rotation3D<>(xaxis, yaxis, zaxis));
        double graspW = MetricUtil::dist2(p1, p2);

        // distance between grasping points is graspW
        // we close gripper such that it is 1 cm more openned than the target1
        Q oq = _openQ;
        oq(0) = std::max(_openQ(0), _closeQ(0)-(graspW+0.01)/2.0);
        oq(0) = std::min(_closeQ(0), oq(0) );
        _gripper->setQ(oq, state);
        
        // then check for collision
        moveFrameW(wTobj * target, _gripperTCP, _gripperMovable, state);
        
        CollisionDetector::QueryResult result;
        if (!cdetect->inCollision(state, &result, true)) {
            ++successes;
            
            GraspTarget gtarget(target);
            gtarget.result = ownedPtr(new GraspResult());
            gtarget.result->testStatus = GraspTask::UnInitialized;
            gtarget.result->objectTtcpTarget = target;
            gtarget.result->gripperConfigurationGrasp = oq;
            stask.addTarget(gtarget);
        }
    }
	
	return gtask;
}
