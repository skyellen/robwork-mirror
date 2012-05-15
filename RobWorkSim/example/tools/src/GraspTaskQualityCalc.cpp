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
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rwsim::dynamics;
using namespace boost::program_options;
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

int calcPerturbedQuality(GraspTask::Ptr gtask, std::string outfile, int pertubations ){
    int count = 0, succCnt=0, failCnt=0;
    std::vector<std::pair<GraspSubTask*,GraspTarget*> > tasks = gtask->getAllTargets();
    GraspTask::Ptr ngtask = gtask->clone();
    for(size_t i = 0; i<tasks.size();i++){
        // save the target that the quality should be calculated for
        if(count==0){
            GraspSubTask nstask = tasks[i].first->clone();
            ngtask->addSubTask( nstask );
            ngtask->getSubTasks().back().addTarget( *tasks[i].second );
            count++;
            continue;
        }

        GraspResult::Ptr tres = tasks[i].second->getResult();
        if(tres->testStatus == GraspTask::Success || tres->testStatus == GraspTask::ObjectSlipped){
            succCnt++;
        } else {
            failCnt++;
        }

        if(count==pertubations){
            // set the quality of the target
            GraspResult::Ptr result = ngtask->getSubTasks().back().getTargets().back().getResult();

            double successProbability = ((double)succCnt)/((double)(succCnt+failCnt));
            result->qualityAfterLifting = Q(1, successProbability );
            succCnt = 0;
            failCnt = 0;
            count = 0;
        } else {
            count++;
        }
    }
    GraspTask::saveRWTask(ngtask, outfile);
    return 0;
}

KDTreeQ* buildKDTree_normal(GraspTask::Ptr gtask,  std::vector<KDTreeQ::KDNode>& simnodes);
KDTreeQ* buildKDTree(GraspTask::Ptr gtask, std::vector<KDTreeQ::KDNode>& simnodes);

Q calculateQuality(ProximityModel::Ptr object, Device::Ptr gripper, CollisionDetector& detector, CollisionStrategy::Ptr strat, State &state, Q openQ, Q closeQ);

double sAreaSum;
std::vector<double> surfaceArea;

int main(int argc, char** argv)
{
    Math::seed(time(NULL));
    srand ( time(NULL) );

    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("output,o", value<string>()->default_value("out.xml"), "the output file.")
        ("oformat,b", value<string>()->default_value("RWTASK"), "The output format, RWTASK, UIBK, Text.")
        ("exclude,e", value<std::vector<string> >(), "Exclude grasps based on TestStatus.")
        ("include,i", value<std::vector<string> >(), "Include grasps based on TestStatus. ")
        ("input", value<string>(), "Name of grasp task file.")
        ("perturbe", value<bool>()->default_value(false), "Add small random pertubations to all targets.")
        ("pertubations", value<int>()->default_value(1), "Number of pertubations to perform on each target.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);

    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

	std::string grasptask_file = vm["input"].as<std::string>();
	std::string outfile = vm["output"].as<std::string>();
	bool perturbe = vm["perturbe"].as<bool>();
	int pertubations = vm["pertubations"].as<int>();


	GraspTask::Ptr gtask = GraspTask::load( grasptask_file );

	if(perturbe){
	    return calcPerturbedQuality(gtask, outfile, pertubations );
	}


    // create nodes for all successes
    //std::vector<KDTree<Pose6D<>, 6 >::KDNode> nodes;
	typedef std::pair<GraspSubTask*, GraspTarget*> Value;
    std::vector<KDTreeQ::KDNode> nodes;
    KDTreeQ *nntree = buildKDTree(gtask, nodes);

    // we need some kind of distance metric
    Q diff(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 15*Deg2Rad);
    std::list<const KDTreeQ::KDNode*> result;
    size_t nodeNr=0;
    BOOST_FOREACH(KDTreeQ::KDNode& node, nodes){
        result.clear();
        Q key  = node.key;
        nntree->nnSearchRect(key-diff,key+diff, result);
        size_t nrNeighbors = result.size();

        GraspResult::Ptr gres = node.valueAs<GraspResult::Ptr>();

        gres->qualityAfterLifting = Q(1, nrNeighbors );

        nodeNr++;
    }


    std::cout << std::endl;


    try {
        GraspTask::saveRWTask( gtask, outfile );
        //GraspTask::saveUIBK( &gtask, outfile );
    } catch (const Exception& exp) {
       RW_WARN("Task Execution Widget: Unable to save tasks");
    }

	return 0;
}


KDTreeQ* buildKDTree_normal(GraspTask::Ptr gtask,  std::vector<KDTreeQ::KDNode>& simnodes) {
    // we need the simulated grasps to attach a quality to the experiments
    // first we build a NN-structure for efficient nearest neighbor search

    BOOST_FOREACH(GraspSubTask& stask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            Transform3D<> t3d = target.pose;
            Vector3D<> n = t3d.R()*Vector3D<>::z();
            Q key(3, n[0], n[1], n[2]);
            simnodes.push_back( KDTreeQ::KDNode(key, std::pair<GraspSubTask*,GraspTarget*>(&stask,&target)) );
        }
    }
    return KDTreeQ::buildTree(simnodes);
}

KDTreeQ* buildKDTree(GraspTask::Ptr gtask, std::vector<KDTreeQ::KDNode>& simnodes) {
    // we need the simulated grasps to attach a quality to the experiments
    // first we build a NN-structure for efficient nearest neighbor search
    BOOST_FOREACH(GraspSubTask& stask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            Transform3D<> t3d = target.getResult()->objectTtcpLift;
            //Transform3D<> t3d = target.pose;
            Vector3D<> p = t3d.P();
            Vector3D<> n = t3d.R()*Vector3D<>::z();
            Q key(6, p[0], p[1], p[2], n[0], n[1], n[2]);

            simnodes.push_back( KDTreeQ::KDNode(key, target.getResult()) );
        }
    }
    return KDTreeQ::buildTree(simnodes);
}

std::vector<Frame*> withColModels(std::vector<Frame*> frames){
    std::vector<Frame*> res;
    BOOST_FOREACH(Frame* frame, frames){
        // TODO change to objects, find them in workcell....
        //if(CollisionModelInfo::get(frame).size()>0)
        //    res.push_back(frame);
    }
    return res;
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
    std::vector<Frame*> jaw1 = withColModels( GeometryUtil::getAnchoredFrames(*ends[0], state) );
    std::vector<Frame*> jaw2 = withColModels( GeometryUtil::getAnchoredFrames(*ends[1], state) );

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

