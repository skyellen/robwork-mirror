
#include <vector>
#include <string>

#include "util.hpp"

#include <rwlibs/task/GraspTask.hpp>

#include <boost/program_options/parsers.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

using namespace std;
using namespace rw::math;
using namespace rwlibs::algorithms;
using namespace rwlibs::task;
using namespace boost::filesystem;
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
    //Unused: int count = vm["samples"].as<int>();
    GraspTask::Ptr gtask = GraspTask::load( input );

    if( boost::filesystem::exists(output) ){
        std::cout << "Output allready exists! skipping filtering." << std::endl;
        return 0;
    }


    // first we build a search tree to efficiently search for targets in 6d
    std::vector<GTaskNNSearch::Node*> simnodes;
    std::cout << "Building tree!" << std::endl;
    GTaskNNSearch *nntree = buildKDTree_pos_eaa(gtask, simnodes);
    std::cout << "sim nodes:"  << simnodes.size() << std::endl;

    Q diff(7, distThres, distThres, distThres, angleThres, angleThres, angleThres,angleThres);
    std::cout << diff << std::endl;
    typedef std::pair<GraspSubTask*, GraspTarget*> Value;
    std::vector< Value > selGrasps;
    std::list<const GTaskNNSearch::Node*> result;

    for(std::size_t i=0;i<simnodes.size(); i++){
        if(simnodes[i]->value.second == NULL)
            continue;
        // find the node with the highest quality
        double q_max = -100;
        GTaskNNSearch::Node *n_max = simnodes[0];

        // TODO: use a priority queue instead of this brute search
        int idx = 0;
        BOOST_FOREACH(GTaskNNSearch::Node* nptr, simnodes){
            GTaskNNSearch::Node& n = *nptr;
            idx++;
            GTaskNNSearch::Value &val = n.value;
            if(val.second==NULL){
                continue;
            }
            if(val.second->result==NULL)
                continue;
            Q qual = val.second->getResult()->qualityAfterLifting;
            if(qual.size()==0)
                qual = val.second->getResult()->qualityBeforeLifting;

            if(qual.size()==0)
                continue;
            if(q_max<qual[0]){
                //std::cout << "found higher:" << qual[0] << " idx: " << idx << std::endl;
                q_max = qual[0];
                n_max = &n;
            }
        }
        // just to make sure we found a node above....
        if(q_max<-90)
            break;
        // we stop the sampling at a certain threshold 30% success
        //if(q_max<0.3)
        //    break;

        // add the max node
        GTaskNNSearch::Value &v = n_max->value;
        selGrasps.push_back( v );
        v.second = NULL;

        // use the max node as a sample point and remove all neighboring nodes
        result.clear();
        nntree->nnSearchRect( n_max->key-diff, n_max->key+diff, result);
        //std::cout << "s:" << result.size() << "\n";
        BOOST_FOREACH(const GTaskNNSearch::Node* res_n, result){
            nntree->removeNode( res_n->key );
            GTaskNNSearch::Node* r_n = (GTaskNNSearch::Node*) res_n;
            r_n->value.second = NULL;
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

    GraspTask::saveRWTask(&res, output);
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
	GTaskNNSearch *nntree_normal = buildKDTree_eaa(gtask);

	// now we sample the normals with random orientations
	Q diff(4, angleThres, angleThres, angleThres, angleThres);
	typedef std::pair<GraspSubTask*, GraspTarget*> Value;

	std::vector< Value > selGrasps;
	std::list<const GTaskNNSearch::Node*> result;
	int samples_f = 0, samples_t = 0;
	while((int)selGrasps.size()<count){
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
        if(idx == (int)result.size())
            idx--;
        std::list<const GTaskNNSearch::Node*>::iterator i = result.begin();
        //std::cout << idx << "==" <<  result.size() << std::endl;
        if(idx>0)
            std::advance(i, idx);
        const GTaskNNSearch::Node* node = *i;
        Value v = node->value;
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
