#include "TaskGenerator.hpp"

#include <vector>
#include <iostream>
#include <rwlibs/algorithms/kdtree/KDTree.hpp>
#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>
#include <rw/geometry/TriMeshSurfaceSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>



using namespace std;

USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;



TaskGenerator::TaskGenerator(TaskDescription::Ptr td) :
	_td(td),
	_tasks(NULL),
	_samples(NULL)
{
	_openQ = Q(1, td->getGripperDevice()->getBounds().second[0]);
	_closeQ = Q(1, td->getGripperDevice()->getBounds().first[0]);
}



void TaskGenerator::moveFrameW(const Transform3D<>& wTtcp, Frame* tcp,
			MovableFrame* base, State& state)
{
    Transform3D<> tcpTbase = Kinematics::frameTframe(tcp, base, state);
    Transform3D<> wTbase_target = wTtcp * tcpTbase;
   
    base->moveTo(wTbase_target, state);
}



Transform3D<> TaskGenerator::_sample(double minDist, double maxDist,
	TriMeshSurfaceSampler& sampler, ProximityModel::Ptr object, ProximityModel::Ptr ray,
	CollisionStrategy::Ptr cstrategy, double &graspW)
{
    // choose a random number in the total area
    TriMesh::Ptr mesh = sampler.getMesh();
    ProximityStrategyData data;
    data.setCollisionQueryType( CollisionStrategy::AllContacts );
    
    bool targetFound = false;
    int tries = 0;
    Transform3D<> target;
    do {
        target = sampler.sample();
        Vector3D<> pos = target.P();
        // z-axis is aligned with tri normal
        Vector3D<> negFaceNormal = -(target.R() * Vector3D<>::z());
        Vector3D<> faceNormal = -negFaceNormal;
        Vector3D<> xaxis = target.R() * Vector3D<>::x();

        // add some noise to the "shooting direction"
        negFaceNormal(0) += Math::ran(-0.1,0.1);
        negFaceNormal(1) += Math::ran(-0.1,0.1);
        negFaceNormal(2) += Math::ran(-0.1,0.1);
        negFaceNormal = normalize( negFaceNormal );

        Rotation3D<> rot(normalize(cross(xaxis,negFaceNormal)), xaxis, negFaceNormal);
        Transform3D<> rayTrans( pos-faceNormal*0.01, rot );

        // now we want to find any triangles that collide with the ray and which are parallel with the sampled
        cstrategy->inCollision(object, Transform3D<>::identity(), ray, rayTrans, data);
        typedef std::pair<int,int> PrimID;
        BOOST_FOREACH(PrimID pid, data.getCollisionData()._geomPrimIds){
			// search for a triangle that has a normal
			Triangle<> tri = mesh->getTriangle( pid.first );
			Vector3D<> normal = tri.calcFaceNormal();
			bool closeAngle = angle(negFaceNormal,normal)<50*Deg2Rad;
			double dist = MetricUtil::dist2(tri[0], pos);
            
		   if (closeAngle) {
				
				// calculate target
				Vector3D<> avgNormal = normalize( (negFaceNormal+normal)/2.0 );
				Vector3D<> xcol = normalize( cross(xaxis,-avgNormal) );
				Vector3D<> ycol = normalize( cross(-avgNormal,xcol) );
				Rotation3D<> rot2( xcol, ycol, -avgNormal);
				Rotation3D<> trot = rot2*RPY<>(Math::ran(0.0,Pi*2.0), 0, 0).toRotation3D();
				// next we rotate z-axis into place
				trot = trot * RPY<>(0, 90*Deg2Rad, 0).toRotation3D();
				target.R() = trot;

				graspW = dist;
				if(dot(tri[0]-pos, -faceNormal)>0)
					target.P() = pos-faceNormal*(dist/2.0);
				else
					target.P() = pos+faceNormal*(dist/2.0);
					
				if (_td->hasHints()) {
					targetFound = false;
				} else {
					targetFound = true;
				}
				// test if the target belongs in the area around hinted grasps
				BOOST_FOREACH (Transform3D<> hint, _td->getHints()) {
					// calculate distance
					double dist = MetricUtil::dist2(target.P(), hint.P());
					double a = angle(target.R()*Vector3D<>::z(), hint.R()*Vector3D<>::z());
					
					if (dist <= _td->getTeachDistance()[0] && a <= _td->getTeachDistance()[1]) {
						targetFound = true;
						break;
					}
				}
				
				if (targetFound) break;
			}
        }
        
        tries++;
        if (tries > 10000) {
			RW_THROW("Cannot find target without collision! Tries: " << tries);
		}
    } while( !targetFound);
    
    return target;
}



rwlibs::task::GraspTask::Ptr TaskGenerator::filterTasks(const rwlibs::task::GraspTask::Ptr tasks, rw::math::Q diff)
{
	if (!tasks) {
		RW_WARN("tasks = NULL");
		return NULL;
	}
	
	// create nodes for succesful grasps
	typedef GraspResult::Ptr ValueType;
	typedef KDTreeQ<ValueType> NNSearch;
	vector<NNSearch::KDNode> nodes;
	//int nTasks = nodes.size();
	
	BOOST_FOREACH(GraspTarget& target, tasks->getSubTasks()[0].getTargets()) {
		if (target.getResult()->testStatus == GraspTask::Success) {
			Q key(7);
            key[0] = target.pose.P()[0];
            key[1] = target.pose.P()[1];
            key[2] = target.pose.P()[2];
            EAA<> eaa(target.pose.R());
            key[3] = eaa.axis()(0);
            key[4] = eaa.axis()(1);
            key[5] = eaa.axis()(2);
            key[6] = eaa.angle();
            
			nodes.push_back(NNSearch::KDNode(key, target.getResult()));
		}
	}
	
	NNSearch *nntree = NNSearch::buildTree(nodes);
    std::list<const NNSearch::KDNode*> result;
    
    int nRemoved = 0;
    BOOST_FOREACH (NNSearch::KDNode& node, nodes) {
		if (node.value->testStatus != GraspTask::TimeOut) {
			result.clear();
			Q key = node.key;
			nntree->nnSearchRect(key-diff, key+diff, result);

			int removed = 0;
			BOOST_FOREACH (const NNSearch::KDNode* n, result) {
				if (n == &node) continue;
				
				if (n->value->testStatus != GraspTask::TimeOut) ++removed;
				const_cast<NNSearch::KDNode*>(n)->value->testStatus = GraspTask::TimeOut;
			}
			nRemoved += removed;
		}
	}
	
	//cout << "Total number of grasps: " << nTasks << " Filtered: " << nTasks - nRemoved << endl;
	
	return tasks;
}



int TaskGenerator::countTasks(const rwlibs::task::GraspTask::Ptr tasks, const rwlibs::task::GraspTask::Status status)
{
	int n = 0;
	
	BOOST_FOREACH (GraspTarget& target, tasks->getSubTasks()[0].getTargets()) {
		if (target.getResult()->testStatus == status)
			++n;
	}
	
	return n;
}



rwlibs::task::GraspTask::Ptr TaskGenerator::generateTask(int nTargets, rw::proximity::CollisionDetector::Ptr cdetect, rw::kinematics::State state)
{
	Transform3D<> wTobj = Kinematics::worldTframe(_td->getTargetObject()->getBase(), state);
	
	// setup task
	GraspTask::Ptr gtask = new GraspTask;
	gtask->getSubTasks().resize(1);
	GraspSubTask& stask = gtask->getSubTasks()[0];
	gtask->setGripperID(_td->getGripperID());
    stask.offset = wTobj;
    stask.approach = Transform3D<>(Vector3D<>(0, 0, 0.1));
    stask.retract = Transform3D<>(Vector3D<>(0, 0, 0.1));
    stask.openQ = _openQ;
    stask.closeQ = _closeQ;
    gtask->setTCPID(_td->getGripperTCP()->getName());
    gtask->setGraspControllerID(_td->getControllerID());
    
    // setup all samples
    _samples = new GraspTask;
    _samples->getSubTasks().resize(1);
	GraspSubTask& atask = _samples->getSubTasks()[0];
    
    // prepare
    TriMeshSurfaceSampler sampler(_td->getTargetObject()->getGeometry()[0]);
	sampler.setRandomPositionEnabled(false);
	sampler.setRandomRotationEnabled(false);
	
	CollisionStrategy::Ptr cstrategy = ProximityStrategyFactory::makeDefaultCollisionStrategy();
    CollisionDetector cd(_td->getWorkCell(), cstrategy);
    
    PlainTriMeshF *rayMesh = new PlainTriMeshF(1);
    (*rayMesh)[0] = Triangle<float>( Vector3D<float>(0,(float)-0.001,0),Vector3D<float>(0,(float)0.001,0),Vector3D<float>(0,0,(float)10) );
    ProximityModel::Ptr ray = cstrategy->createModel();
    Geometry geom(rayMesh); // we have to wrap the trimesh in an geom object
    geom.setId("Ray");
    ray->addGeometry(geom);
	
	ProximityModel::Ptr object = cstrategy->createModel();
    cstrategy->addGeometry(object.get(), _td->getTargetObject()->getGeometry()[0]);
    
    for (int successes = 0; successes < nTargets;) {
		double graspW = 0.0;
		Transform3D<> target = _sample(_closeQ[0]*2.0, _openQ[0]*2.0, sampler, object, ray, cstrategy, graspW);

        // distance between grasping points is graspW
        // we close gripper such that it is 1 cm more openned than the target
        Q oq = _openQ;
        oq(0) = std::max(_closeQ(0), _closeQ(0)+(graspW+0.01)/2.0);
        oq(0) = std::min(_openQ(0), oq(0) );
        _td->getGripperDevice()->setQ(oq, state);
        //cout << oq << endl;
        
        // then check for collision
        moveFrameW(wTobj * target, _td->getGripperTCP(), _td->getGripperMovable(), state);
        
        CollisionDetector::QueryResult result;
        if (!cdetect->inCollision(state, &result, true)) {
            ++successes;
            
            GraspTarget gtarget(target);
            gtarget.result = ownedPtr(new GraspResult());
            gtarget.result->testStatus = GraspTask::UnInitialized;
            gtarget.result->objectTtcpTarget = target;
            //gtarget.result->gripperConfigurationGrasp = oq;
            stask.addTarget(gtarget);
            atask.addTarget(gtarget);
        } else {
			//cout << "collision" << endl;
			GraspTarget gtarget(target);
            gtarget.result = ownedPtr(new GraspResult());
            gtarget.result->testStatus = GraspTask::Success;
            gtarget.result->objectTtcpTarget = target;
            gtarget.result->gripperConfigurationGrasp = oq;
            atask.addTarget(gtarget);
		}
    }
    
    _tasks = gtask;
    
	return gtask;
}
