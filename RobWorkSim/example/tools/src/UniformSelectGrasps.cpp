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

//#include <rwsim/dynamics/ContactManifold.hpp>
//#include <rwsim/dynamics/ContactPoint.hpp>
//#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

//#include <rwsim/dynamics/DynamicUtil.hpp>

#include <rw/geometry/GeometryFactory.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <iterator>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace boost::program_options;

variables_map init(int argc, char** argv){
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "This script filters a grasp list by iteratively selecting the best grasp "
                 "and removing all neighboring grasps within a 6D cube arounf the selected "
                 "grasp. The 6D cube is defined by angle and dist arguments.")
        ("output,o", value<string>()->default_value("out.xml"), "the output file.")
        ("angle", value<double>()->default_value(30.0), "Angle of cone in degree.")
        ("dist", value<double>()->default_value(0.01), "Dist of square in meters.")
        ("samples", value<int>()->default_value(500), "Number of grasps to sample.")
        ("input", value<string>(), "The input grasp task file that should be filtered.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);

    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

    // write standard welcome, status
    if (vm.count("help")) {
        cout << "Usage:\n\n"
                  << "\t" << argv[0] <<" [options] -o<outfile> <expFile1> <expFile2> <...> <expFileN> \n"
                  << "\n";
        cout << desc << "\n";
        abort();
    }

    return vm;
}

KDTreeQ* buildKDTree_normal(GraspTask::Ptr gtask);
KDTreeQ* buildKDTree(GraspTask::Ptr gtask, std::vector<KDTreeQ::KDNode>& simnodes);

int main_jaj(int argc, char** argv);
int main_lpe(int argc, char** argv);

int main(int argc, char** argv){
    //return main_jaj(argc,argv);
    return main_lpe(argc,argv);
}

/**
 * We need to choose a number of grasps
 */
int main_lpe(int argc, char** argv)
{
    Math::seed(time(NULL));
    srand ( time(NULL) );
    variables_map vm = init(argc, argv);

    std::string grasptask_file_out = vm["output"].as<std::string>();
    double angleThres = vm["angle"].as<double>()*Deg2Rad;
    double distThres = vm["dist"].as<double>();
    std::string input = vm["input"].as<string>();
    std::string output = vm["output"].as<string>();
    int count = vm["samples"].as<int>();
    GraspTask::Ptr gtask = GraspTask::load( input );

    // first we build a search tree to efficiently search for targets in 6d
    std::vector<KDTreeQ::KDNode> simnodes;
    KDTreeQ *nntree = buildKDTree(gtask,simnodes);


    Q diff(7, distThres, distThres, distThres, angleThres, angleThres, angleThres,angleThres);
    typedef std::pair<GraspSubTask*, GraspTarget*> Value;
    std::vector< Value > selGrasps;
    std::list<const KDTreeQ::KDNode*> result;

    for(int i=0;i<simnodes.size(); i++){
        // find the node with the highest quality
        double q_max = -100;
        KDTreeQ::KDNode *n_max = &simnodes[0];
        BOOST_FOREACH(KDTreeQ::KDNode& n, simnodes){
            Value &val = n.valueAs<Value&>();
            if(val.second==NULL)
                continue;
            if(val.second->getResult()==NULL)
                continue;
            Q qual = val.second->getResult()->qualityAfterLifting;
            if(qual.size()==0)
                continue;
            if(q_max<qual[0]){
                q_max = qual[0];
                n_max = &n;
            }
        }
        if(q_max<-90)
            break;
        // we stop the sampling at a certain threshold 30% success
        if(q_max<0.3)
            break;

        // add the max node
        Value &v = n_max->valueAs<Value&>();
        selGrasps.push_back( v );
        v.second = NULL;

        // use the max node as a sample point and remove all neighboring nodes
        result.clear();
        nntree->nnSearchRect( n_max->key-diff, n_max->key+diff, result);
        BOOST_FOREACH(const KDTreeQ::KDNode* res_n, result){
            KDTreeQ::KDNode* r_n = (KDTreeQ::KDNode*) res_n;
            r_n->valueAs<Value&>().second = NULL;
        }
    }
    std::cout << "GENRATED: "<< selGrasps.size() << " from " << simnodes.size() << std::endl;

    // convert selectGrasps to a graspTask
    GraspTask res = *gtask;
    res.getSubTasks().clear();
    BOOST_FOREACH(Value& taskpair, selGrasps){
        GraspSubTask stask = *taskpair.first;
        stask.targets.clear();
        stask.targets.push_back( *taskpair.second );
        res.addSubTask(stask);
    }

    GraspTask::saveRWTask(&res, output + ".rwtask.xml");
    //std::cout << "COVERAGE: " << (((double)samples_t)/(double)samples_f)*100.0 << "%" << std::endl;
    return 0;
}

/**
 * We need to choose a number of grasps
 */
int main_jaj(int argc, char** argv)
{
    Math::seed(time(NULL));
    srand ( time(NULL) );
    variables_map vm = init(argc, argv);

	std::string grasptask_file_out = vm["output"].as<std::string>();
	double angleThres = vm["angle"].as<double>()*Deg2Rad;
    std::string input = vm["input"].as<string>();
    std::string output = vm["output"].as<string>();
    int count = vm["samples"].as<int>();
	GraspTask::Ptr gtask = GraspTask::load( input );

	// first we build a search tree to efficiently search for targets orientet in a certain direction
	KDTreeQ *nntree_normal = buildKDTree_normal(gtask);

	// now we sample the normals with random orientations
	Q diff(4, angleThres, angleThres, angleThres, angleThres);
	typedef std::pair<GraspSubTask*, GraspTarget*> Value;

	std::vector< Value > selGrasps;
	std::list<const KDTreeQ::KDNode*> result;
	int samples_f = 0, samples_t = 0;
	while(selGrasps.size()<count){
        result.clear();
        EAA<> eaa( Math::ranRotation3D<double>() );
        //Vector3D<> k = Math::ranRotation3D<double>()*Vector3D<>::z();
        Vector3D<> k = eaa.axis();
        Q key(3, k[0], k[1], k[2], eaa.angle());
        nntree_normal->nnSearchRect(key-diff,key+diff, result);
        samples_f++;
        if(result.size()==0)
            continue;
        samples_t++;
        int idx = Math::ranI(0,result.size());
        if(idx == result.size())
            idx--;
        std::list<const KDTreeQ::KDNode*>::iterator i = result.begin();
        //std::cout << idx << "==" <<  result.size() << std::endl;
        if(idx>0)
            std::advance(i, idx);
        const KDTreeQ::KDNode* node = *i;
        Value v = node->valueAs<Value>();
        selGrasps.push_back( v );
        std::cout << result.size() << "    \r" << std::flush;
	}
	// print all failures

	// convert selectGrasps to a graspTask
	GraspTask res = *gtask;
	res.getSubTasks().clear();
	BOOST_FOREACH(Value& taskpair, selGrasps){
	    GraspSubTask stask = *taskpair.first;
	    stask.targets.clear();
	    stask.targets.push_back( *taskpair.second );
	    res.addSubTask(stask);
	}

	GraspTask::saveRWTask(&res, output + ".rwtask.xml");
	std::cout << "COVERAGE: " << (((double)samples_t)/(double)samples_f)*100.0 << "%" << std::endl;
	return 0;
}


KDTreeQ* buildKDTree_normal(GraspTask::Ptr gtask) {
    // we need the simulated grasps to attach a quality to the experiments
    // first we build a NN-structure for efficient nearest neighbor search
    std::vector<KDTreeQ::KDNode> *simnodes = new std::vector<KDTreeQ::KDNode>();
    BOOST_FOREACH(GraspSubTask& stask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            Transform3D<> t3d = target.pose;
            //Vector3D<> n = t3d.R()*Vector3D<>::z();
            EAA<> eaa( t3d.R() );
            Vector3D<> n = eaa.axis();
            Q key(4, n[0], n[1], n[2], eaa.angle());
            simnodes->push_back( KDTreeQ::KDNode(key, std::pair<GraspSubTask*,GraspTarget*>(&stask,&target)) );
        }
    }
    return KDTreeQ::buildTree(*simnodes);
}

KDTreeQ* buildKDTree(GraspTask::Ptr gtask, std::vector<KDTreeQ::KDNode>& simnodes) {
    // we need the simulated grasps to attach a quality to the experiments
    // first we build a NN-structure for efficient nearest neighbor search
    BOOST_FOREACH(GraspSubTask& stask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            Transform3D<> t3d = target.pose;
            Vector3D<> p = t3d.P();
            //Vector3D<> n = t3d.R()*Vector3D<>::z();
            EAA<> eaa( t3d.R() );
            Vector3D<> n = eaa.axis();

            Q key(7, p[0], p[1], p[2], n[0], n[1], n[2], eaa.angle() );
            simnodes.push_back( KDTreeQ::KDNode(key, std::pair<GraspSubTask*,GraspTarget*>(&stask,&target)) );
        }
    }
    return KDTreeQ::buildTree(simnodes);
}

