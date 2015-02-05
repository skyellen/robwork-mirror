
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>
#include <vector>

#include <rw/rw.hpp>
#include <rw/math/Vector3D.hpp>
#include <rwlibs/task.hpp>
#include <rwlibs/task/GraspTask.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace boost::program_options;

int main(int argc, char** argv)
{
    // we need
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("gdb1", value<string>(), "the grasp db to find equal grasps against. ")
        ("gdb2", value<string>(), "the grasp db that should be matched against db1.")
    ;
    positional_options_description optionDesc;

    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);


    // write standard welcome, status
    if (vm.count("help")) {
        cout << "Usage:\n\n"
                  << "\t" << argv[0] <<" [options] --gdb1=db1 --gdb2=db2 \n"
                  << "\n";

        cout << desc << "\n";
        return 1;
    }

    using namespace boost::filesystem;
    std::string gdb1_name = vm["gdb1"].as<string>();
    std::string gdb2_name = vm["gdb2"].as<string>();
    GraspTask::Ptr gdb1 = GraspTask::load( gdb1_name );
    GraspTask::Ptr gdb2 = GraspTask::load( gdb2_name );


    std::vector<std::pair<GraspSubTask*,GraspTarget*> > gdb1_targets = gdb1->getAllTargets();
    std::vector<std::pair<GraspSubTask*,GraspTarget*> > gdb2_targets = gdb2->getAllTargets();

    Metric<Transform3D<double> >::Ptr t3ddist = MetricFactory::makeTransform3DMetric<double>(1.0,1/(90*Deg2Rad));

    std::vector<int> gdb1ToGdb2(gdb1_targets.size(), -1);
    for(int i=0;i<gdb1_targets.size();i++){

    	for(int j=0;j<gdb2_targets.size();j++){
    		// compare target i with target j
    		if( t3ddist->distance(gdb1_targets[i].second->pose, gdb2_targets[i].second->pose)<0.00001 ){
    			gdb1ToGdb2[i] = j;
    			break;
    		}
    	}
    	if(gdb1ToGdb2[i]<0){
    		gdb1ToGdb2[i] = 1000000;
    	}
    	std::cout << i << "\t" << gdb1ToGdb2[i] << "\n";
    }



    return 0;
}

