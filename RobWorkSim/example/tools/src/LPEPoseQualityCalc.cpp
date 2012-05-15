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

//#include <rwsim/dynamics/ContactPoint.hpp>
//#include <rwsim/dynamics/ContactCluster.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/math/Vector3D.hpp>
/*
#include <rwsim/dynamics/ContactManifold.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>
*/
#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
/*
#include <rwsim/dynamics/DynamicUtil.hpp>
#include <rwsim/dynamics/ContactManifold.hpp>
*/
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
//using namespace rwsim::dynamics;
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


std::vector<std::pair<Transform3D<>, RPY<> > > readPoses(std::string file){

    std::vector<std::pair<Transform3D<>, RPY<> > > data;
    char line[1000];
    std::ifstream in(file.c_str());
    in.getline(line,1000);
    in.getline(line,1000);
    in.getline(line,1000);

    char tmpc;
    while(!in.eof() ){
        in.getline(line,1000);
        std::istringstream istr(line);
        Vector3D<float> pos_s, pos_e;
        EAA<float> rot_s, rot_e;
        RPY<float> rpy;

        sscanf(line,"%f %f %f %f %f %f %f  %f %f ",
               &pos_s[0], &pos_s[1], &pos_s[2], &rpy(0), &rpy(1), &rpy(2), &rot_s[0], &rot_s[1], &rot_s[2]);


        //std::cout << pos_e << std::endl;
        data.push_back( std::make_pair(Transform3D<>(cast<double>(pos_s),cast<double>(rot_s)), cast<double>(rpy) ) );

    }
    std::cout << "Size of intput: " << data.size() << std::endl;
    return data;
}


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
        ("stable", value<string>(), "Name of grasp task file.")
        ("misses", value<string>(), "Add small random pertubations to all targets.")
        ("angle", value<double>(), "angular standard deviation.")
        //("dist", value<double>(), "distance standard deviation.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);

    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

	std::string stable_file = vm["stable"].as<std::string>();
	std::string misses_file = vm["misses"].as<std::string>();
    std::string output_file = vm["output"].as<std::string>();
    double angle_sd = vm["angle"].as<double>()*Deg2Rad;


    std::vector<std::pair<Transform3D<>,RPY<> > > stable = readPoses(stable_file);
    std::vector<std::pair<Transform3D<>,RPY<> > > misses = readPoses(misses_file);
    std::vector<double> qualityestimates( stable.size() );
    std::vector<double> qualityestimates_misses( misses.size() );
    // build the nodes
    typedef std::pair<int, bool> Value;
    std::vector<KDTreeQ::KDNode> nodes;
    for(int i=0;i<stable.size();i++){
        Transform3D<> t3d = stable[i].first;
        //stable[i].second(1) = stable[i].second(1)+13*Deg2Rad;
        RPY<> rpy = stable[i].second;
        Q key(2, rpy(1), rpy(2));
        nodes.push_back( KDTreeQ::KDNode(key, Value(i, true)) );
    }
    for(int i=0;i<misses.size();i++){
        Transform3D<> t3d = misses[i].first;
        //misses[i].second(1) = misses[i].second(1)+13*Deg2Rad;
        RPY<> rpy = misses[i].second;
        Q key(2, rpy(1), rpy(2));
        nodes.push_back( KDTreeQ::KDNode(key, Value(i, false)) );
    }

    // create nodes for all successes
    //std::vector<KDTree<Pose6D<>, 6 >::KDNode> nodes;
    KDTreeQ *nntree = KDTreeQ::buildTree( nodes );


    // define the truncation
    Q diff(2, 2*angle_sd, 2*angle_sd);

    // now for each stable node predict its quality
    std::list<const KDTreeQ::KDNode*> result;
    result.clear();

    BOOST_FOREACH(KDTreeQ::KDNode &nn, nodes){
    //for(int i=0;i<stable.size();i++){
        result.clear();
        Transform3D<> t3d;
        RPY<> rpy ;
        Value nv = nn.valueAs<Value>();
        if(nv.second){
            t3d = stable[nv.first].first;
            rpy = stable[nv.first].second;
        } else {
            t3d = misses[nv.first].first;
            rpy = misses[nv.first].second;
        }
        Q key(2, rpy(1), rpy(2));
        nntree->nnSearchRect(key-diff,key+diff, result);

        double sum = 0, sum1=0;
        double xo = rpy(1);
        double yo = rpy(2);

        int N = result.size();
        BOOST_FOREACH(const KDTreeQ::KDNode* n, result ){
            Value v = n->valueAs<Value>();
            RPY<> rpyo;
            if(v.second)
                rpyo = stable[v.first].second;
            else
                rpyo = misses[v.first].second;

            double x = rpyo(1);
            double y = rpyo(2);

            // calculate the
            double tmp_x = Math::sqr(x-xo)/(2*Math::sqr(angle_sd));
            double tmp_y = Math::sqr(y-yo)/(2*Math::sqr(angle_sd));


            double val = std::pow(2.718281828, -(tmp_x+tmp_y));
            //std::cout << val << " " <<  N << " y-yo" << y-yo << std::endl;
            //std::cout << val << " " <<  N << " x-xo" << x-xo << std::endl;
            if(v.second) // if o=1
                sum += val;
            sum1 += val;
        }


        double quality = sum/sum1;

        //std::cout << sum << " " << sum1 << " "<< quality <<  std::endl;
        if(nv.second){
            qualityestimates[nv.first] = quality;
        } else {
            qualityestimates_misses[nv.first] = quality;
        }
    }

    std::ofstream outf(output_file.c_str());

    for(int i=0;i<stable.size();i++){
        Transform3D<> t3d = stable[i].first;
        EAA<> eaa(t3d.R());
        RPY<> rpy = stable[i].second;
        outf << t3d.P()[0] << "\t" << t3d.P()[1] << "\t" << t3d.P()[2] << "\t";
        outf << rpy(0)*Rad2Deg << "\t" << rpy(1)*Rad2Deg << "\t" << rpy(2)*Rad2Deg << "\t";
        outf << qualityestimates[i] << "\n";
    }
    RW_WARN("1");
    for(int i=0;i<misses.size();i++){
        Transform3D<> t3d = misses[i].first;
        EAA<> eaa(t3d.R());
        RPY<> rpy = misses[i].second;
        outf << t3d.P()[0] << "\t" << t3d.P()[1] << "\t" << t3d.P()[2] << "\t";
        outf << rpy(0)*Rad2Deg << "\t" << rpy(1)*Rad2Deg << "\t" << rpy(2)*Rad2Deg << "\t";
        outf << qualityestimates_misses[i] << "\n";
    }

    outf.close();

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



const Q normalize(const Q& v)
{
    double length = v.norm2();
    if (length != 0)
        return v/length;
    else
        return Q::zero(v.size());
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

