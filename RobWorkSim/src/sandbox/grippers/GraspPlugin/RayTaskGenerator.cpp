#include "RayTaskGenerator.hpp"

#include <rw/geometry/TriMeshSurfaceSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>



using namespace std;

USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;




RayTaskGenerator::RayTaskGenerator(rwsim::dynamics::DynamicWorkCell::Ptr dwc, const std::string& objectID, const std::string& gripperID) :
	TaskGenerator(dwc, objectID, gripperID)
{
}



Transform3D<> RayTaskGenerator::_sample(double minDist, double maxDist, TriMeshSurfaceSampler& sampler, ProximityModel::Ptr object, ProximityModel::Ptr ray, CollisionStrategy::Ptr cstrategy, double &graspW)
{
    // now we choose a random number in the total area
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
            double dist = MetricUtil::dist2(tri[0], pos);//fabs( dot(faceNormal, tri[0]-pos) );
            bool closeDist = minDist<dist && dist<maxDist;
            
            //cout << minDist << " " << dist << " " << maxDist << endl;
            //cout << " " << closeAngle << endl;
            
            if (closeAngle && closeDist) {
                targetFound = true;
                // calculate target

                //target.R() = rot*RPY<>(Math::ran(0.0,Pi), 0, 0).toRotation3D();
                //target.R() = rot;
                // the target transform needs to have the z-axis pointing into the x-y-plane
                // soooo, we first generate some randomness
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
                break;
            }
        }
        
        tries++;
        if (tries > 100) {
			RW_THROW("Cannot find target without collision!");
		}
    } while( !targetFound);
    
    return target;
}



rwlibs::task::GraspTask::Ptr RayTaskGenerator::generateTask(int nTargets, rw::proximity::CollisionDetector::Ptr cdetect, rw::kinematics::State state)
{
	//State state = _wc->getDefaultState();
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
    
    // prepare
    TriMeshSurfaceSampler sampler(_object->getGeometry()[0]);
	sampler.setRandomPositionEnabled(false);
	sampler.setRandomRotationEnabled(false);
	
	CollisionStrategy::Ptr cstrategy = ProximityStrategyFactory::makeDefaultCollisionStrategy();
    CollisionDetector cd(_wc, cstrategy);
    
    PlainTriMeshF *rayMesh = new PlainTriMeshF(1);
    (*rayMesh)[0] = Triangle<float>( Vector3D<float>(0,(float)-0.001,0),Vector3D<float>(0,(float)0.001,0),Vector3D<float>(0,0,(float)10) );
    ProximityModel::Ptr ray = cstrategy->createModel();
    Geometry geom(rayMesh); // we have to wrap the trimesh in an geom object
    geom.setId("Ray");
    ray->addGeometry(geom);
	
	ProximityModel::Ptr object = cstrategy->createModel();
    cstrategy->addGeometry(object.get(), _object->getGeometry()[0]);
    
    for (int successes = 0; successes < nTargets;) {
		double graspW = 0.0;
		Transform3D<> target = _sample(_closeQ[0]*2.0+_jawdist, _openQ[0]*2.0+_jawdist, sampler, object, ray, cstrategy, graspW);

        // distance between grasping points is graspW
        // we close gripper such that it is 1 cm more openned than the target
        Q oq = _openQ;
        oq(0) = std::max(_closeQ(0), _closeQ(0)+(graspW+0.01)/2.0);
        oq(0) = std::min(_openQ(0), oq(0) );
        _gripper->setQ(oq, state);
        cout << oq << endl;
        
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
        } else {
			//cout << "collision" << endl;
		}
    }
	
	return gtask;
}

