#include <sstream>
#include <fstream>
#include <vector>
#include <string>

#include <rw/geometry/GeometryUtil.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/algorithms/kdtree/KDTree.hpp>
#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>
#include <rwlibs/task/GraspTask.hpp>

using namespace std;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rwlibs::algorithms;
using namespace rwlibs::task;

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

    if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of grasp experiments task file" << std::endl;
	    std::cout << "- Arg 2 name of simulated grasp task file" << std::endl;
	    std::cout << "- Arg 3 name of output txt file\n" << std::endl;
	    return 0;
	}

	std::string grasptask_file(argv[1]);
	std::string sim_grasptask_file( argv[2] );
	std::string outfile(argv[3]);

	std::ofstream out(outfile.c_str());

	GraspTask::Ptr gtask_exp = GraspTask::load( grasptask_file );
	GraspTask::Ptr gtask_sim = GraspTask::load( sim_grasptask_file );

	// we need the simulated grasps to attach a quality to the experiments
	// first we build a NN-structure for efficient nearest neighbor search

	//Q scale(7,1.0/0.1,1.0/0.1,1.0/0.1,1.0/2.0,1.0/2.0,1.0/2.0,1.0/Pi);
	Q scale(7,1.0,1.0,1.0,1.0,1.0,1.0,1.0);

	typedef KDTreeQ<GraspResult::Ptr> NNSearchRes;
	typedef KDTreeQ<NNSearchRes::Node*> NNSearchPos;
    std::vector<NNSearchRes::Node> simnodes;
    std::vector<NNSearchPos::Node> posnodes;

    BOOST_FOREACH(GraspSubTask& stask, gtask_sim->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            if(target.result==NULL)
                continue;
            if(target.result->testStatus!=GraspResult::Success && target.result->testStatus!=GraspResult::ObjectSlipped)
                continue;
            if(target.result->qualityAfterLifting[0]<0.05)
                continue;

            Transform3D<> t3d = target.pose;

            Q key(6);
            key[0] = t3d.P()[0];
            key[1] = t3d.P()[1];
            key[2] = t3d.P()[2];
            EAA<> eaa(t3d.R());
            key[3] = eaa.axis()(0);
            key[4] = eaa.axis()(1);
            key[5] = eaa.axis()(2);
            //key[6] = eaa.angle();
            //for(size_t i=0;i<key.size(); i++)
            //    key[i] = key[i] * scale[i];

            simnodes.push_back( NNSearchRes::Node(key, target.result) );
            Q pos(3);
            pos[0] = t3d.P()[0];
            pos[1] = t3d.P()[1];
            pos[2] = t3d.P()[2];

            posnodes.push_back( NNSearchPos::Node(pos, &simnodes.back()) );
        }
    }

    NNSearchRes *nntree = NNSearchRes::buildTree(simnodes);
    //Unused: NNSearchPos *nntree_pos = NNSearchPos::buildTree(posnodes);

    //Q diff(7, 0.015, 0.015, 0.015, 25*Deg2Rad, 25*Deg2Rad, 25*Deg2Rad, 25*Deg2Rad);
    Q diff(6, 0.005, 0.005, 0.005, 8*Deg2Rad, 8*Deg2Rad, 8*Deg2Rad);
    std::cout << "DIFF: "<<diff<<std::endl;
    //for(size_t i=0;i<diff.size(); i++)
    //    diff[i] = diff[i] * scale[i];

    std::list<const NNSearchRes::KDNode*> result;
    size_t maxNeigh = 0;
    // for each simulated node calculate an extra quality based on nr of close successes
    BOOST_FOREACH(NNSearchRes::KDNode& node, simnodes){
        result.clear();
        GraspResult::Ptr res = node.value;
        Q key = node.key;
        nntree->nnSearchRect(key-diff,key+diff, result);
        size_t nrNeighbors = result.size();
        Q qualities(res->qualityAfterLifting.size()+1);
        for(size_t i=0;i<res->qualityAfterLifting.size();i++)
            qualities[i] = res->qualityAfterLifting[i];
        maxNeigh = std::max(nrNeighbors,maxNeigh);
        qualities[res->qualityAfterLifting.size()] = nrNeighbors;
        res->qualityAfterLifting = qualities;
    }
    std::cout << "MAX NEIGHBORS: " << maxNeigh << std::endl;

    std::cout << "DIFF: "<<diff<<std::endl;

    // normalize to [0;1]
    RW_WARN("2");
    BOOST_FOREACH(NNSearchRes::KDNode& node, simnodes){
        GraspResult::Ptr res = node.value;
        res->qualityAfterLifting[res->qualityAfterLifting.size()-1] *= (1.0/maxNeigh);
    }

    std::cout << "DIFF: "<<diff<<std::endl;

    //KDTreeQ *nntree = KDTreeQ::buildTree(simnodes);
    RW_WARN("2");
    // now
    std::vector<std::string> successes;
    std::vector<std::string> failures;
    size_t q_dim = 0;
	BOOST_FOREACH(GraspSubTask& stask, gtask_exp->getSubTasks()){
	    BOOST_FOREACH(GraspTarget& target,stask.targets ){
	        std::stringstream ss;
	        Transform3D<> t3d = target.pose;


            Q key(6);
            key[0] = t3d.P()[0];
            key[1] = t3d.P()[1];
            key[2] = t3d.P()[2]; // scale it to be in 0..1 size
            EAA<> eaa(t3d.R());
            key[3] = eaa.axis()(0);
            key[4] = eaa.axis()(1);
            key[5] = eaa.axis()(2);
            //key[6] = eaa.angle();
            Q key_scaled = key;
            for(size_t i=0;i<key_scaled.size(); i++)
                key_scaled[i] = key[i] * scale[i];


            result.clear();
            GraspResult::Ptr gres = target.getResult(); //node.valueAs<GraspResult::Ptr>();

            nntree->nnSearchRect(key-diff,key+diff, result);
            std::cout << key-diff << std::endl;
            std::cout << key+diff << std::endl;

            size_t nrNeighbors = result.size();
            std::cout << "nrNeighbors: " << nrNeighbors << std::endl;
            int exptestStatus = gres->testStatus;
            gres->testStatus = GraspResult::Success;

            Q quality(gres->qualityAfterLifting.size()+1);
            for(std::size_t i=0;i<gres->qualityAfterLifting.size();i++)
                quality[i] = gres->qualityAfterLifting[i];

            quality[gres->qualityAfterLifting.size()] = nrNeighbors*(1.0/maxNeigh);
            gres->qualityAfterLifting = quality;

            Q key_pos = key.getSubPart(0,3);

            // find the simnode closest to the experiment
            //KDTreeQ::KDNode& node =  nntree->nnSearch(key);

            //Unused: double closest_dist = 1000.0;
            NNSearchRes::Node* closest_node = NULL;
            // make bruteforce search on position alone

            /*
            BOOST_FOREACH(KDTreeQ::KDNode& closenode, simnodes){

                double ndist = MetricUtil::dist2(closenode.key.getSubPart(0,3), key.getSubPart(0,3));
                if(ndist<closest_dist){
                    closest_dist = ndist;
                    closest_node = (KDTreeQ::KDNode*) &closenode;
                }
            }
            */

            //KDTreeQ::KDNode* closest_node_pos =  &nntree_pos->nnSearch(key_pos);
            //closest_node = closest_node_pos->valueAs<KDTreeQ::KDNode*>();

            closest_node =  &nntree->nnSearch(key);
            //result.clear();
            //GraspResult::Ptr res = node.valueAs<GraspResult::Ptr>();
            //nntree->nnSearchRect(key-diff,key+diff, result);

            /*
            if(result.size()==0){
                closest_node =  &nntree->nnSearch(key);
            } else {
                BOOST_FOREACH(const KDTreeQ::KDNode* closenode, result){
                    double ndist = MetricUtil::dist2(closenode->key, key);
                    if(ndist<closest_dist){
                        closest_dist = ndist;
                        closest_node = (KDTreeQ::KDNode*) closenode;
                    }
                }
            }
            */


            GraspResult::Ptr gressim = closest_node->value;
            double dist = MetricUtil::dist2(closest_node->key,key);
            std::cout << (closest_node->key-key) << std::endl;
            Q qual = gressim->qualityAfterLifting;
            q_dim = qual.size();
            if((exptestStatus==GraspResult::Success) || (exptestStatus==GraspResult::ObjectSlipped)){
                ss << "1\t"<< dist;
                Q qTmp;
                qTmp = gres->qualityAfterLifting;
                for (std::size_t ind = 0; ind < qTmp.size(); ind++) ss << "\t" << qTmp[ind];
                qTmp = key_scaled;
                for (std::size_t ind = 0; ind < qTmp.size(); ind++) ss << "\t" << qTmp[ind];
                successes.push_back(ss.str());
            } else {
                ss << "0\t"<< dist;
                Q qTmp;
                qTmp = gres->qualityAfterLifting;
                for (std::size_t ind = 0; ind < qTmp.size(); ind++) ss << "\t" << qTmp[ind];
                qTmp = key_scaled;
                for (std::size_t ind = 0; ind < qTmp.size(); ind++) ss << "\t" << qTmp[ind];
                failures.push_back(ss.str());
            }
	    }
	}


	// print all failures

	out << "success\tdist";
	for(size_t i=0;i<q_dim;i++){ out << "\tqual"<<i; }
	out << "\tx\ty\tz\teaax\teaay\teaaz\teaa_a\n";
	BOOST_FOREACH(const std::string& str, successes){ out << str << "\n";}
	BOOST_FOREACH(const std::string& str, failures){ out << str << "\n";}

	out.close();

	GraspTask::saveUIBK(gtask_exp, outfile + ".uibk.xml");

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

