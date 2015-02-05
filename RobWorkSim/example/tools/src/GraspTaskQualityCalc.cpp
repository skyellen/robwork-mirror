#include <iostream>
#include <vector>
#include <string>

#include "util.hpp"

#include <rw/rw.hpp>
#include <rwlibs/task/GraspTask.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace boost::program_options;
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

Transform3D<> sampleParSurface(double minDist, double maxDist, TriMesh::Ptr mesh, ProximityModel::Ptr object, ProximityModel::Ptr ray, CollisionStrategy::Ptr cstrategy, double &graspW);


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
        if(tres->testStatus == GraspResult::Success || tres->testStatus == GraspResult::ObjectSlipped){
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

	std::string input = vm["input"].as<std::string>();
	std::string outfile = vm["output"].as<std::string>();
	bool perturbe = vm["perturbe"].as<bool>();
	int pertubations = vm["pertubations"].as<int>();

	path outp(outfile);
	create_directory(outp);

    path ip(input);
    std::vector<std::string> infiles;
    if( is_directory(ip) ){
        infiles = IOUtil::getFilesInFolder( ip.string(), false, true);
    } else {
        infiles.push_back( ip.string() );
    }

	if(perturbe){
	    BOOST_FOREACH(std::string file, infiles){
	        std::stringstream sstr;
	        GraspTask::Ptr gtask = GraspTask::load( file );
	        std::cout << "Processing: " << path(file).filename().string() << std::endl;
	        sstr << outfile << "/" << path(file).filename().string();
	        calcPerturbedQuality(gtask, sstr.str(), pertubations );
	    }
	    return 0;
	}

    GraspTask::Ptr gtask = GraspTask::load( input );

    // create nodes for all successes
    //std::vector<KDTree<Pose6D<>, 6 >::KDNode> nodes;
	typedef std::pair<GraspSubTask*, GraspTarget*> Value;
    std::vector<GTaskNNSearch::KDNode> nodes;
    GTaskNNSearch *nntree = buildKDTree_pos_zaxis(gtask, nodes);

    // we need some kind of distance metric
    Q diff(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 15*Deg2Rad);
    std::list<const GTaskNNSearch::KDNode*> result;
    size_t nodeNr=0;
    BOOST_FOREACH(GTaskNNSearch::KDNode& node, nodes){
        result.clear();
        Q key  = node.key;
        nntree->nnSearchRect(key-diff,key+diff, result);
        size_t nrNeighbors = result.size();

        GraspResult::Ptr gres = node.value.second->getResult();
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

