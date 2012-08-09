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

#include <rw/geometry/TriMeshSurfaceSampler.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <rwlibs/task/GraspTask.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;

using namespace boost::program_options;
using namespace boost::numeric::ublas;
using namespace boost::filesystem;


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

Transform3D<> sampleParSurface(double minDist, double maxDist, TriMeshSurfaceSampler& sampler, ProximityModel::Ptr object, ProximityModel::Ptr ray, CollisionStrategy::Ptr cstrategy, double &graspW);

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

    // we need
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("wc", value<string>()->default_value(""), "The workcell.")
        ("object", value<string>(), "then object name or stl file")
        ("gripper", value<string>(), "the gripper.")
        ("output,o", value<string>()->default_value("out.xml"), "the output file.")
        ("oformat,b", value<string>()->default_value("RWTASK"), "The output format, RWTASK, UIBK, Text.")
        ("open", value<double>(), "default will be max q of gripper.")
        ("close", value<double>(), "default will be min q of gripper.")
        ("jawdist", value<double>(), "The distance between jaw 1 and jaw 2 when closed.")
        ("samples", value<int>()->default_value(2000), "Nr of grasp samples that should be generated.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);

    // initialize RobWork log. We put debug into rwdebug.log and warning into rwwarn.log
    //Log::log().setWriter(Log::Debug, ownedPtr( new LogFileWriter("rwdebug.log") ) );
    //Log::log().setWriter(Log::Warning, ownedPtr( new LogFileWriter("rwwarn.log") ) );



    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

    rw::math::Math::seed( TimerUtil::currentTimeMs() );
    // write standard welcome, status
    if (vm.count("help")) {
        cout << "Usage:\n\n"
                  << "\t" << argv[0] <<" [options] -o<outfile> <expFile1> <expFile2> <...> <expFileN> \n"
                  << "\n";
        cout << desc << "\n";
        return 1;
    }



    const int NR_OF_SAMPLES = vm["samples"].as<int>();

    path file_wc(vm["wc"].as<std::string>());
    if( !exists(file_wc) ){ RW_ASSERT("WorkCell option \"wc\" must be a valid file name!"); }


    // load workcell
    std::cout << "Loading workcell: " << file_wc.string() << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::load( file_wc.string() );
    RW_ASSERT( wc!=NULL );

    std::string name_object = vm["object"].as<std::string>();
    path file_object(name_object);
    Geometry::Ptr object_geo;
    if( exists(file_object) ){
        // load geometry from file
        object_geo = GeometryFactory::getGeometry(file_object.string());
    }

    if( object_geo==NULL ){
        // load it from workcell
        Object::Ptr obj = wc->findObject(name_object);
        if( obj==NULL  || obj->getGeometry().size()==0 ){
            RW_ASSERT("Object option \"object\" must be a valid file name or a name of an object in the workcell description!");
        }
        object_geo = obj->getGeometry()[0];
    }


	std::string outfile = vm["output"].as<std::string>();
	std::string grippername = vm["gripper"].as<std::string>();
	Geometry::Ptr geo = object_geo;

	Device::Ptr gripper = wc->findDevice(grippername);
	RW_ASSERT(gripper!=NULL);

	// setup openq and closeq
	double OPENQ = gripper->getBounds().second[0];
	double CLOSEQ = gripper->getBounds().first[0];
	if(vm.count("open")>0){ OPENQ = vm["open"].as<double>(); }
	if(vm.count("close")>0){ OPENQ = vm["close"].as<double>(); }
	if(vm.count("jawdist")==0) RW_THROW("jawdist option must be specified!");
	double jawdist = vm["jawdist"].as<double>();

    Q openQ(1,OPENQ);
    Q closeQ(1,CLOSEQ);

	// TODO: this should be automized
	Frame* gripperTCP = wc->findFrame( "GRIPPER_TCP" );
	RW_ASSERT(gripperTCP!=NULL);
	MovableFrame* gripperMovable = wc->findFrame<MovableFrame>( "GRIPPER_MOVER" );
	RW_ASSERT(gripperMovable!=NULL);

	TriMeshSurfaceSampler sampler(geo);
	sampler.setRandomPositionEnabled(false);
	sampler.setRandomRotationEnabled(false);

	std::string type = gripper->getName();

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



    //wTe_n = Transform3D<>::identity();
    //wTe_home = Transform3D<>::identity();
    gtask.setGripperID(type);
    stask.offset = wTe_n;
    stask.approach = Transform3D<>(Vector3D<>(0,0,0.04));
    stask.retract = Transform3D<>(Vector3D<>(0,0,0.10));
    stask.openQ = openQ;
    stask.closeQ = closeQ;
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

    // create another sampling method based on point pair features
    std::vector< std::pair<Vector3D<>,Vector3D<> > > points;
    std::cout << "creating points..." << std::endl;
    // first sample 10000
    for(int i=0;i<10000;i++){
        Transform3D<> transform = sampler.sample();
        points.push_back( std::make_pair(transform.P(),transform.R()*Vector3D<>::z()) );
    }
    // now create a feature list where normals are <50Deg apart and
    std::cout << "creating features" << std::endl;
    std::vector<std::pair<int,int> > features;
    for(int i=0;i<10000; i++){
        std::pair<Vector3D<>,Vector3D<> > &p1 = points[i];
        for(int j=i;j<10000;j++){
            std::pair<Vector3D<>,Vector3D<> > &p2 = points[j];
            bool closeAngle = angle(-p1.second,p2.second)<50*Deg2Rad;
            double dist = MetricUtil::dist2(p1.first, p2.first);
            if(closeAngle && (dist > CLOSEQ+jawdist) && (dist < OPENQ*2.0+jawdist)){
                features.push_back( std::make_pair(i,j) );
            }
        }
    }
    std::cout << "features created, nr features: " << features.size() << std::endl;



    // also add the stl object
    ProximityModel::Ptr object = cstrategy->createModel();
    cstrategy->addGeometry(object.get(), geo);
    State state = wc->getDefaultState();

    int tries =0;

    // create nodes for all successes
    //std::vector<KDTree<Pose6D<>, 6 >::KDNode> nodes;
    typedef GraspResult::Ptr ValueType;
    typedef KDTreeQ<ValueType> NNSearch;
    std::vector<NNSearch::KDNode> nodes;
    std::vector<NNSearch::KDNode> allnodes;
    int nrSuccesses = 0;
    for(int i=0; nrSuccesses<NR_OF_SAMPLES; i++){
        CollisionDetector::QueryResult result;
        if( !( i%10) )
            std::cout << "Successes: " << nrSuccesses << " of "  << i <<"     \r" << std::flush;

        double graspW = 0;

        //Transform3D<> target = sampleParSurface(CLOSEQ+jawdist,OPENQ*2.0+jawdist, sampler, object, ray, cstrategy, graspW);

        int i = Math::ranI(0, features.size()-1);
        std::pair<int,int> sfeat = features[i];
        Vector3D<> p1 = points[sfeat.first].first;
        Vector3D<> p2 = points[sfeat.second].first;
        Vector3D<> tcp_p = (p2-p1)/2.0 + p1;
        // generate orientation, xaxis determine gripper closing direction, zaxis the approach
        Vector3D<> xaxis = normalize(p2-p1);
        Vector3D<> yaxis = normalize(cross(Vector3D<>(Math::Math::ranNormalDist(0,1),Math::Math::ranNormalDist(0,1),Math::Math::ranNormalDist(0,1)), xaxis ));
        Vector3D<> zaxis = normalize( cross(xaxis,yaxis) );
        Transform3D<> target( tcp_p, Rotation3D<>(xaxis,yaxis,zaxis) );
        graspW = MetricUtil::dist2(p1,p2);
        //std::cout << graspW << std::endl;
        //if(!LinearAlgebra::isProperOrthonormal(target.R().m() ))
        //    RW_WARN("NO PROPER TRANSFORM");

        // distance between grasping points is graspW
        // we close gripper such that it is 1 cm more openned than the target

        Q oq = openQ;
        oq(0) = std::min(openQ(0),(graspW+0.01)/2.0);
        oq(0) = std::max(closeQ(0), oq(0) );
        gripper->setQ( oq, state);
/*
        Quaternion<> quat( target.R() );
        quat.normalize();
        target.R() = quat.toRotation3D();
*/
        // place gripper in target position
        moveFrameW(target, gripperTCP, gripperMovable, state);
        if( cdetect.inCollision(state, &result, true) ){

            tries++;

            GraspTarget gtarget( target );
            gtarget.result = ownedPtr( new GraspResult() );
            gtarget.result->testStatus = GraspTask::CollisionInitially;
            gtarget.result->objectTtcpTarget = target;
            gtarget.result->gripperConfigurationGrasp = oq;
            //stask.addTarget( gtarget );

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
            allnodes.push_back( NNSearch::KDNode(key, gtarget.result) );

        } else {
            nrSuccesses++;
            GraspTarget gtarget( target );
            gtarget.result = ownedPtr( new GraspResult() );
            gtarget.result->testStatus = GraspTask::UnInitialized;
            gtarget.result->objectTtcpTarget = target;
            gtarget.result->gripperConfigurationGrasp = oq;
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
            nodes.push_back( NNSearch::KDNode(key, gtarget.result) );
            allnodes.push_back( NNSearch::KDNode(key, gtarget.result) );

            //Q quality = calculateQuality(object, gripper, cdetect, cstrategy, state, openQ, closeQ);
            //gtarget.result->qualityAfterLifting

        }

    }
    std::cout << std::endl;
    std::cout << "Building search trees... ";
    NNSearch *nntree = NNSearch::buildTree(nodes);
    NNSearch *nntree_all = NNSearch::buildTree(allnodes);

    std::cout << std::endl;

    // we need some kind of distance metric
    Q diff(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 15*Deg2Rad);
    std::list<const NNSearch::KDNode*> result;
    size_t nodeNr=0;
    BOOST_FOREACH(NNSearch::KDNode& node, nodes){
        result.clear();
        Q key  = node.key;
        nntree->nnSearchRect(key-diff,key+diff, result);
        size_t nrNeighbors = result.size();
        result.clear();
        nntree_all->nnSearchRect(key-diff,key+diff, result);
        size_t nrNeighbors_all = result.size();

        GraspResult::Ptr gres = node.value;
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
        node.value->qualityAfterLifting = Q(1, nrNeighbors );

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


Transform3D<> sampleParSurface(double minDist, double maxDist, TriMeshSurfaceSampler& sampler, ProximityModel::Ptr object, ProximityModel::Ptr ray, CollisionStrategy::Ptr cstrategy, double &graspW){
    // now we choose a random number in the total area
    TriMesh::Ptr mesh = sampler.getMesh();
    ProximityStrategyData data;
    data.setCollisionQueryType( CollisionStrategy::AllContacts );
    bool targetFound = false;
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
        cstrategy->inCollision(object,Transform3D<>::identity(), ray, rayTrans, data);
        typedef std::pair<int,int> PrimID;
        BOOST_FOREACH(PrimID pid, data.getCollisionData()._geomPrimIds){
            // search for a triangle that has a normal
            Triangle<> tri = mesh->getTriangle( pid.first );
            Vector3D<> normal = tri.calcFaceNormal();
            bool closeAngle = angle(negFaceNormal,normal)<40*Deg2Rad;
            double dist = MetricUtil::dist2(tri[0], pos);//fabs( dot(faceNormal, tri[0]-pos) );
            bool closeDist = minDist<dist && dist<maxDist;
            if(closeAngle && closeDist){
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
    } while( !targetFound );
    return target;
}

