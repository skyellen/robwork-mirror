#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwlibs/algorithms/kdtree/KDTree.hpp>
#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>

#include <vector>
#include <rwlibs/task/GraspTask.hpp>
#include <rw/geometry/STLFile.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/math/Vector3D.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <rwsim/dynamics/DynamicUtil.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rwsim::dynamics;
using namespace rwlibs::task;

using namespace boost::numeric::ublas;


const double SOFT_LAYER_SIZE = 0.0005;

int binSearchRec(const double value, std::vector<double>& surfaceArea, size_t start, size_t end){
    if(start==end)
        return start;
    // choose a int between start and end
    size_t split = (end-start)/2+start;

    if(value<surfaceArea[split])
        return binSearchRec(value, surfaceArea, start, split);
    else
        return binSearchRec(value, surfaceArea, split+1, end);
}

Transform3D<> sampleParSurface(double minDist, double maxDist, TriMesh::Ptr mesh, ProximityModel::Ptr object, ProximityModel::Ptr ray, CollisionStrategy::Ptr cstrategy, double &graspW);

void moveFrameW(const Transform3D<>& wTtcp, Frame *tcp, MovableFrame* base, State& state){
    Transform3D<> tcpTbase = Kinematics::frameTframe(tcp, base, state);
    Transform3D<> wTbase_target = wTtcp * tcpTbase;
    //base->setTransform(wTbase_target, state);
    base->moveTo(wTbase_target, state);
    //std::cout << wTbase_target << std::endl;
    //std::cout << Kinematics::worldTframe(base, state) << std::endl;
}

Q calculateQuality(ProximityModel::Ptr object, Device::Ptr gripper, CollisionDetector& detector, CollisionStrategy::Ptr strat, State &state, Q openQ, Q closeQ);

double sAreaSum;
std::vector<double> surfaceArea;

int main(int argc, char** argv)
{
    Math::seed(time(NULL));
    srand ( time(NULL) );

    if( argc < 5 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of stl file" << std::endl;
	    std::cout << "- Arg 2 name of output xml file\n" << std::endl;
	    std::cout << "- Arg 3 name of workcell file" << std::endl;
	    std::cout << "- Arg 4 name of gripper" << std::endl;

	    return 0;
	}



	std::string filename(argv[1]);
	std::string outfile(argv[2]);
	std::string workcellfile(argv[3]);
	std::string grippername(argv[4]);
	Geometry::Ptr geo;

	WorkCell::Ptr wc = WorkCellLoader::load( workcellfile );
	//wc->getPropertyMap().get<std::string>("GRIPPER_TCP");

	Device::Ptr gripper = wc->findDevice(grippername);
	RW_ASSERT(gripper!=NULL);
	Frame* gripperTCP = wc->findFrame( "GRIPPER_TCP" );
	RW_ASSERT(gripperTCP!=NULL);
	MovableFrame* gripperMovable = wc->findFrame<MovableFrame>( "GRIPPER_MOVER" );
	RW_ASSERT(gripperMovable!=NULL);

	try{
        geo = GeometryFactory::getGeometry(filename);
	} catch (...){
	    RW_WARN("No such file: \""<< filename << "\"");
	    return 0;
	}
	TriMesh::Ptr mesh = geo->getGeometryData()->getTriMesh();
	surfaceArea = std::vector<double>(mesh->size());
	sAreaSum = 0;
	// run through mesh and create the search
	for(size_t i=0; i<mesh->size(); i++){
	    Triangle<> tri = mesh->getTriangle(i);
	    // calculate triangle area
	    sAreaSum += tri.calcArea();
	    surfaceArea[i] = sAreaSum;
	}
	std::string type = "GS20";
	// these should be the object transformation
    Vector3D<> pos(0, 0, 0);
    Rotation3D<> rot(1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);

    GraspTask gtask;
    gtask.getSubTasks().resize(1);
    GraspSubTask &stask = gtask.getSubTasks()[0];
    //rwlibs::task::CartesianTask::Ptr tasks = gtask.getRootTask();
    // first set up the configuration
    Vector3D<> d(0,0,-0.02);
    Transform3D<> wTe_n(pos, rot);
    Transform3D<> wTe_home(pos+inverse(rot)*d, rot);

    RW_ASSERT( fabs(LinearAlgebra::det( wTe_n.R().m() ))-1.0 < 0.0001 );
    RW_ASSERT( fabs(LinearAlgebra::det( wTe_home.R().m() ))-1.0 < 0.0001 );

    //const double OPENQ = 0.035;
    const double OPENQ = 0.005;
    const double CLOSEQ = 0.0005;


    Q openQ(1,OPENQ);
    Q closeQ(1,CLOSEQ);

    //wTe_n = Transform3D<>::identity();
    //wTe_home = Transform3D<>::identity();
    gtask.setGripperID(type);
    stask.offset = wTe_n;
    if( type== "SCUP"){
        stask.approach = Transform3D<>(Vector3D<>(0,0,0.04));
    } else {
        stask.approach = Transform3D<>(Vector3D<>(0,0,0.04));
    }
    stask.openQ = openQ;
    stask.closeQ = closeQ;
    //gtask.setTCPID("GS20TCP");
    gtask.setTCPID("GRIPPER_TCP");
    gtask.setGraspControllerID("GraspController");

    CollisionStrategy::Ptr cstrategy = ProximityStrategyFactory::makeDefaultCollisionStrategy();
    CollisionDetector cdetect(wc, cstrategy);
    PlainTriMeshF *rayMesh = new PlainTriMeshF(1);
    (*rayMesh)[0] = Triangle<float>( Vector3D<float>(0,(float)-0.001,0),Vector3D<float>(0,(float)0.001,0),Vector3D<float>(0,0,(float)10) );

    ProximityModel::Ptr ray = cstrategy->createModel();
    Geometry geom( rayMesh ); // we have to wrap the trimesh in an geom object
    geom.setId("Ray");
    ray->addGeometry(geom);

    // also add the stl object
    ProximityModel::Ptr object = cstrategy->createModel();
    cstrategy->addGeometry(object.get(), geo);
    State state = wc->getDefaultState();

    const int NR_OF_SAMPLES = 2000; int tries =0;

    // create nodes for all successes
    //std::vector<KDTree<Pose6D<>, 6 >::KDNode> nodes;
    std::vector<KDTreeQ::KDNode> nodes;
    std::vector<KDTreeQ::KDNode> allnodes;

    for(int i=0; i<NR_OF_SAMPLES; i++){
        CollisionDetector::QueryResult result;
        std::cout << "Target: " << i << "  "  << tries <<"     \r" << std::flush;
        double graspW = 0;
        Transform3D<> target = sampleParSurface(CLOSEQ+0.03,OPENQ*2.0+0.03, mesh, object, ray, cstrategy, graspW);
        RW_ASSERT( fabs(LinearAlgebra::det( target.R().m() ))-1.0 < 0.0001 );

        Q oq = openQ;
        oq(0) = (graspW+0.01)/2.0;
        gripper->setQ( oq, state);
        // place gripper in target position
        moveFrameW(target, gripperTCP, gripperMovable, state);
        if( cdetect.inCollision(state, &result, true) ){
            tries++;
            GraspTarget gtarget( target );
            gtarget.result = ownedPtr( new GraspResult() );
            gtarget.result->testStatus = GraspTask::CollisionInitially;
            gtarget.result->objectTtcpTarget = target;
            stask.addTarget( gtarget );

            Q key(7);
            key[0] = target.P()[0];
            key[1] = target.P()[1];
            key[2] = target.P()[2];
            EAA<> eaa(target.R());
            key[3] = eaa.axis()(0);
            key[4] = eaa.axis()(1);
            key[5] = eaa.axis()(2);
            key[6] = eaa.angle();

            //nodes.push_back( KDTreeQ::KDNode(key, gtarget.result) );
            allnodes.push_back( KDTreeQ::KDNode(key, gtarget.result) );

        } else {
            GraspTarget gtarget( target );
            gtarget.result = ownedPtr( new GraspResult() );
            gtarget.result->testStatus = GraspTask::UnInitialized;
            gtarget.result->objectTtcpTarget = target;
            stask.addTarget( gtarget );

            // calculate the quality
            // here we try to estimate the contacting area
            Q key(7);
            key[0] = target.P()[0];
            key[1] = target.P()[1];
            key[2] = target.P()[2];
            EAA<> eaa(target.R());
            key[3] = eaa.axis()(0);
            key[4] = eaa.axis()(1);
            key[5] = eaa.axis()(2);
            key[6] = eaa.angle();

            //bool success=true;
            nodes.push_back( KDTreeQ::KDNode(key, gtarget.result) );
            allnodes.push_back( KDTreeQ::KDNode(key, gtarget.result) );

            //Q quality = calculateQuality(object, gripper, cdetect, cstrategy, state, openQ, closeQ);
            //gtarget.result->qualityAfterLifting

        }

    }
    std::cout << std::endl;
    std::cout << "Building search trees... ";
    KDTreeQ *nntree = KDTreeQ::buildTree(nodes);
    KDTreeQ *nntree_all = KDTreeQ::buildTree(allnodes);

    std::cout << std::endl;

    // we need some kind of distance metric
    Q diff(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 15*Deg2Rad);
    std::list<const KDTreeQ::KDNode*> result;
    size_t nodeNr=0;
    BOOST_FOREACH(KDTreeQ::KDNode& node, nodes){
        result.clear();
        Q key  = node.key;
        nntree->nnSearchRect(key-diff,key+diff, result);
        size_t nrNeighbors = result.size();
        result.clear();
        nntree_all->nnSearchRect(key-diff,key+diff, result);
        size_t nrNeighbors_all = result.size();

        GraspResult::Ptr gres = node.valueAs<GraspResult::Ptr>();
        /*
        // count how many that are close rotationally to
        size_t nrClose = 0;
        BOOST_FOREACH(const KDTreeQ::KDNode* nn, result){

            EAA<> eaa(gres->objectTtcpTarget.R()*inverse( nn->valueAs<GraspResult::Ptr>()->objectTtcpTarget.R()) );
            if( eaa.angle() < 20*Deg2Rad)
                nrClose++;
        }
        std::cout << "\n" << nodeNr << "\t" << nrNeighbors << "\t"<< nrClose << std::flush;
        node.valueAs<GraspResult::Ptr>()->qualityAfterLifting = Q(1, nrClose*1.0);
        */

        std::cout << "\n" << nodeNr << "\t" << nrNeighbors << "\t" <<  nrNeighbors_all << std::flush;
        node.valueAs<GraspResult::Ptr>()->qualityAfterLifting = Q(1, nrNeighbors );

        nodeNr++;
    }


    std::cout << std::endl;


    try {
        GraspTask::saveRWTask( &gtask, outfile );
        //GraspTask::saveUIBK( &gtask, outfile );
    } catch (const Exception& exp) {
       RW_WARN("Task Execution Widget: Unable to save tasks");
    }

	return 0;
}

const Q normalize(const Q& v)
{
    double length = v.norm2();
    if (length != 0)
        return v/length;
    else
        return Q::zero(v.size());
}

// we need to estimate the quality of a kinematically generated grasp
// here we try closing the grippers until penetration of SOFT_LAYER_SIZE is reached.
// when that is done then the area of a box fitted to the contacts is used as the quality estimate.
Q calculateQuality(ProximityModel::Ptr object, Device::Ptr grip, CollisionDetector& detector, CollisionStrategy::Ptr strat, State &state, Q openQ, Q closeQ){
    Q result(1,0.0);

    // find the jaws of the gripper
    TreeDevice* gripper = dynamic_cast<TreeDevice*>(grip.get());
    std::vector<Frame*> ends = gripper->getEnds();
    RW_ASSERT(ends.size()==2);
    std::vector<Frame*> jaw1 = GeometryUtil::getAnchoredFrames(*ends[0], state);
    std::vector<Frame*> jaw2 = GeometryUtil::getAnchoredFrames(*ends[1], state);

    gripper->setQ( openQ, state);
    // reduce openQ until a collision is found
    Q stepQ = closeQ - openQ;
    Q ustepQ = normalize( stepQ );
    for(int i=1; i<10; i++){
        gripper->setQ( openQ+stepQ/(2*i), state );
        if( !detector.inCollision(state,NULL, true)){
            openQ = openQ+stepQ/(2*i);
        }
    }
    std::cout << "CQ:" << openQ << std::endl;

    // now one of the jaws should be
    gripper->setQ( openQ+ustepQ*SOFT_LAYER_SIZE, state );
    // we should be in collision now

    // calculate the contacts with jaw1 and jaw2
    //calcContacts( );


    return result;
}


Transform3D<> sampleParSurface(double minDist, double maxDist, TriMesh::Ptr mesh, ProximityModel::Ptr object, ProximityModel::Ptr ray, CollisionStrategy::Ptr cstrategy, double &graspW){
    // now we choose a random number in the total area
    ProximityStrategyData data;
    data.setCollisionQueryType( CollisionStrategy::AllContacts );
    bool targetFound = false;
    Transform3D<> target;
    do {
        double rnum = Math::ran(0.0, sAreaSum);
        int triIds = binSearchRec(rnum, surfaceArea, 0, mesh->size()-1);
        Triangle<> tri = mesh->getTriangle(triIds);

        // random sample the triangle
        double b0 = Math::ran();
        double b1 = ( 1.0f - b0 ) * Math::ran();
        double b2 = 1 - b0 - b1;
        Vector3D<> pos = tri[0] * b0 + tri[1] * b1 + tri[2] * b2;

        //double r1 = Math::ran(), r2 = Math::ran();
        //Vector3D<> pos = (1 - sqrt(r1)) * tri[0] + (sqrt(r1) * (1 - r2)) * tri[1] + (sqrt(r1) * r2) * tri[2];

        Vector3D<> faceNormal = tri.calcFaceNormal();
        // create orientation that point in the -z-axis direction
        Vector3D<> tanV = pos-tri[0];
        if(tanV.norm2()<0.000001)
            tanV = pos-tri[1];
        if(tanV.norm2()<0.000001)
            tanV = pos-tri[2];
        if(tanV.norm2()<0.000001)
            continue;

        tanV = normalize(tanV);
        faceNormal(0) += Math::ran(-0.1,0.1);
        faceNormal(1) += Math::ran(-0.1,0.1);
        faceNormal(2) += Math::ran(-0.1,0.1);
        faceNormal = normalize( faceNormal );
        Rotation3D<> rot(cross(tanV,-faceNormal), tanV, -faceNormal);
        Transform3D<> rayTrans( pos+faceNormal, rot );
        // now we want to find
        cstrategy->inCollision(object,Transform3D<>::identity(), ray, rayTrans, data);
        typedef std::pair<int,int> PrimID;
        BOOST_FOREACH(PrimID pid, data.getCollisionData()._geomPrimIds){
            // search for a triangle that has a normal
            Triangle<> tri = mesh->getTriangle( pid.first );
            Vector3D<> normal = tri.calcFaceNormal();
            bool closeAngle = angle(-faceNormal,normal)<20*Deg2Rad;
            double dist = fabs( dot(faceNormal, tri[0]-pos) );
            bool closeDist = minDist<dist && dist<maxDist;
            if(closeAngle && closeDist){
                targetFound = true;
                // calculate target

                //target.R() = rot*RPY<>(Math::ran(0.0,Pi), 0, 0).toRotation3D();
                //target.R() = rot;
                // the target transform needs to have the z-axis pointing into the x-y-plane
                // soooo, we first generate some randomness
                Vector3D<> avgNormal = (-faceNormal+normal)/2.0;
                Rotation3D<> rot2(cross(tanV,-avgNormal), tanV, -avgNormal);
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
    } while( !targetFound );
    return target;
}

