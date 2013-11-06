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
	Transform3D<> wTobj = Kinematics::worldTframe(_td->getTargetObject()->getBase(), _td->getInitState());
	
	//RW_WARN("SAMPLE");
    // choose a random number in the total area
    TriMesh::Ptr mesh = sampler.getMesh();
    ProximityStrategyData data;
    data.setCollisionQueryType( CollisionStrategy::AllContacts );
    
    bool targetFound = false;
    int tries = 0;
    Transform3D<> target;
    do { 
		//RW_WARN("SAMPLELOOP");
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
        //RW_WARN("1");
        BOOST_FOREACH(PrimID pid, data.getCollisionData()._geomPrimIds){
			//RW_WARN("1.5");
			// search for a triangle that has a normal
			
			//cout << "!!!" << mesh->getSize() << endl;
			//cout << "!!!" << pid.first << endl;
			Triangle<> tri = mesh->getTriangle( pid.first );
			//RW_WARN("1.75");
			Vector3D<> normal = tri.calcFaceNormal();
			
			bool closeAngle = angle(negFaceNormal,normal)<50*Deg2Rad;
			double dist = MetricUtil::dist2(tri[0], pos);
            
            //RW_WARN("2");
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
				//RW_WARN("3");
				// test if the target belongs in the area around hinted grasps
				Transform3D<> targetW = inverse(wTobj) * target;
				
				BOOST_FOREACH (Transform3D<> hint, _td->getHints()) {
					// calculate distance
					Q teachDist = _td->getTeachDistance();
					
					bool distOk = std::fabs(targetW.P()[0] - hint.P()[0]) <= teachDist[0] &&
						std::fabs(targetW.P()[1] - hint.P()[1]) <= teachDist[1] &&
						std::fabs(targetW.P()[2] - hint.P()[2]) <= teachDist[2];
						
					/*EAA<> targetEAA(target.R());
					EAA<> hintEAA(hint.R());
					double R = 1.0 * sin(0.25 * teachDist[3]);
					//cout << R << endl;
					
					bool angleOk = std::fabs(targetEAA.axis()[0] - hintEAA.axis()[0]) <= R &&
						std::fabs(targetEAA.axis()[1] - hintEAA.axis()[1]) <= R &&
						std::fabs(targetEAA.axis()[2] - hintEAA.axis()[2]) <= R &&
						std::fabs(targetEAA.angle() - hintEAA.angle()) <= teachDist[4];
						
					//bool angleOk = std::fabs(angle(targetEAA.axis(), hintEAA.axis())) <= teachDist[3] &&
					//	std::fabs(targetEAA.angle() - hintEAA.angle()) <= teachDist[4];
						
					// find such rotation that z axes are aligned in both target and hint
					/* THIS IS OKAYISH: */
					Transform3D<> aligned(RPY<>(0, angle(targetW.R()*Vector3D<>::z(),hint.R()*Vector3D<>::z()), 0).toRotation3D() * hint.R());
						
					bool angleOk = std::fabs(angle(target.R()*Vector3D<>::z(),hint.R()*Vector3D<>::z())) <= _td->getTeachDistance()[3] &&
						std::fabs(angle(target.R()*Vector3D<>::x(), aligned.R()*Vector3D<>::x())) <= _td->getTeachDistance()[4];
						//std::fabs(angle(target.R()*Vector3D<>::x(), hint.R()*Vector3D<>::x())) <= _td->getTeachDistance()[4];
					/* END OF OKAYISH */
					
					/* DEBUG OUTPUTS FROM NOW ON */
					//std::cout << angleOk << " Z= " << Rad2Deg*std::fabs(angle(target.R()*Vector3D<>::z(),hint.R()*Vector3D<>::z())) <<
					//	" X= " << Rad2Deg*std::fabs(angle(target.R()*Vector3D<>::x(), aligned.R()*Vector3D<>::x())) << std::endl;
						
					/*if (angleOk) {
						std::cout << "hint: " << hintEAA.axis() << " / " << hintEAA.angle() << std::endl;
						std::cout << "target: " << targetEAA.axis() << " / " << targetEAA.angle() << std::endl;
						std::cout << Rad2Deg*angle(targetEAA.axis(), hintEAA.axis()) << std::endl;
					}*/
						
					if (distOk && angleOk) {
						//cout << hintEAA.axis() << " " << targetEAA.axis() << endl;
						//RW_WARN("WEE");
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
    } while (!targetFound);
    
    return target;
}



rwlibs::task::GraspTask::Ptr TaskGenerator::filterTasks(const rwlibs::task::GraspTask::Ptr tasks, rw::math::Q diff)
{
	if (!tasks) {
		//RW_WARN("tasks = NULL");
		return NULL;
	}
	
	// create nodes for succesful grasps
	typedef GraspResult::Ptr ValueType;
	typedef KDTreeQ<ValueType> NNSearch;
	vector<NNSearch::KDNode> nodes;
	//int nTasks = nodes.size();
	
	typedef std::pair<class GraspSubTask*, class GraspTarget*> TaskTarget;
	BOOST_FOREACH (TaskTarget p, tasks->getAllTargets()) {
	//BOOST_FOREACH(GraspTarget& target, tasks->getSubTasks()[0].getTargets()) {
		if (p.second->getResult()->testStatus == GraspTask::Success) {
			Q key(7);
            key[0] = p.second->pose.P()[0];
            key[1] = p.second->pose.P()[1];
            key[2] = p.second->pose.P()[2];
            EAA<> eaa(p.second->pose.R());
            key[3] = eaa.axis()(0);
            key[4] = eaa.axis()(1);
            key[5] = eaa.axis()(2);
            key[6] = eaa.angle();
            
            //cout << key << endl;
            
			nodes.push_back(NNSearch::KDNode(key, p.second->getResult()));
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
			//nntree->nnSearchElipse(key, diff, result);

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
	
	typedef std::pair<class GraspSubTask*, class GraspTarget*> TaskTarget;
	BOOST_FOREACH (TaskTarget p, tasks->getAllTargets()) {
	//BOOST_FOREACH (GraspTarget* target, tasks->getAllTargets().second) { //tasks->getSubTasks()[0].getTargets()) {
		if (p.second->getResult()->testStatus == status)
			++n;
	}
	
	return n;
}



rwlibs::task::GraspTask::Ptr TaskGenerator::generateTask(int nTargets, rw::kinematics::State state)
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
    //RW_WARN("SETUP TASK");
    
    // setup all samples
    _samples = new GraspTask;
    _samples->getSubTasks().resize(1);
	GraspSubTask& atask = _samples->getSubTasks()[0];
    
    // prepare
    TriMeshSurfaceSampler sampler(_td->getTargetObject()->getGeometry()[0]);
	sampler.setRandomPositionEnabled(false);
	sampler.setRandomRotationEnabled(false);
	
	//cout << _td->getTargetObject()->getGeometry()[0]->getName() << endl;
	//cout << "PTR: " << _td->getTargetObject()->getGeometry()[0]->getGeometryData()->getTriMesh() << endl;
	//cout << "MESHSIZE: " << sampler.getMesh() << endl;
	
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
    
    // make CD for WC & add rules excluding interference objects
    CollisionDetector::Ptr cdetect = new CollisionDetector(_td->getWorkCell(), ProximityStrategyFactory::makeDefaultCollisionStrategy());
    BOOST_FOREACH (Object::Ptr obj, _td->getInterferenceObjects()) {
		cdetect->addRule(ProximitySetupRule::makeExclude("gripper.Base", obj->getBase()->getName()));
		cdetect->addRule(ProximitySetupRule::makeExclude("gripper.LeftFinger", obj->getBase()->getName()));
		cdetect->addRule(ProximitySetupRule::makeExclude("gripper.RightFinger", obj->getBase()->getName()));
	}
	//RW_WARN("PREPARE");
    
    for (int successes = 0; successes < nTargets;) {
		//RW_WARN("LOOPY");
		double graspW = 0.0;
		Transform3D<> target = _sample(_closeQ[0]*2.0, _openQ[0]*2.0, sampler, object, ray, cstrategy, graspW);
		//RW_WARN("FOUND");

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
        //RW_WARN("COLLISIONCHECK");
        if (!cdetect->inCollision(state, &result, true)) {
			//RW_WARN("YAY");
            ++successes;
            
            // make new subtask
            GraspSubTask subtask;
            subtask.offset = wTobj;
			subtask.approach = Transform3D<>(Vector3D<>(0, 0, 0.1));
			subtask.retract = Transform3D<>(Vector3D<>(0, 0, 0.1));
			subtask.openQ = oq;
			subtask.closeQ = _closeQ;
            
            GraspTarget gtarget(target);
            gtarget.result = ownedPtr(new GraspResult());
            gtarget.result->testStatus = GraspTask::UnInitialized;
            gtarget.result->objectTtcpTarget = target;
            subtask.addTarget(gtarget);
            gtask->addSubTask(subtask);
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

