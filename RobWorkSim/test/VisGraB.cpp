
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>

#include <vector>

#include <rw/geometry/STLFile.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>
#include <rwsim/dynamics/ContactManifold.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>


USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace boost::program_options;

/*
 * 1. We provide code including the following
- XML input file containing all the scenes that will be used to create grasps
- Code to read that XML, go through all the scenes, and call the user's grasp-generation method
- The three utility functions, that get a vector with grasp (each of three representations) and that adds text to the experiment.XML file
- The resulting experiment.xml file will contain all grasps for all scenes

2. An executable / script that does the following
- Read the experiment.xml file
- Parse that file and create merged_tasks.xml files
- Run RobWorkStudio, going through all the merged_tasks files, restarting when crashed, until all results have been acquired
- Parsing all result files and writing one big visgrab_result.xml file
- Parsing all result files and writing a matlab data file that can be used by the provided matlab scripts to display graphs, make tex/html tables.
 *
 *
 */


int main(int argc, char** argv)
{
    // we need
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("benchmark,b", value<string>(), "The benchmark description file.")
        ("experiments,e", value<vector<string> >(), "File containing a set of VisGraB experiments descriptions.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("experiments",-1);


    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

    if (vm.count("help")) {
        cout << "Usage:\n\n"
                  << "\t" << argv[0] <<" [options] <experimentFile1> <experimentFile2> <...> <experimentFileN> \n"
                  << "\n";

        cout << desc << "\n";
        return 1;
    }

    if (vm.count("experiments")) {
        cout << "experiment file: " << vm["experiments"].as<vector<string> >()[0] << " nr of files:" << vm["experiments"].as<vector<string> >().size() << std::endl;
        // check that the file(s) exists.

        // convert experiments to RobWork tasks


    } else {
        cout << "No experiments specified, please try again and specify a valid VisGraB experiment file.\n";
        return 1;
    }

    return (0);
}


//CarteseanTask::Ptr readExperiments(const std::string& filename){
    // parse the VisGraB experiments format and translate it into the RobWork Task format


//}



