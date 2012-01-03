
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
#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>
#include <rwsim/dynamics/ContactManifold.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rwsim/simulator/GraspTaskSimulator.hpp>
#include <rwsim/simulator/GraspTask.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

#include <rwsim/simulator/GraspTask.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace boost::program_options;
using namespace rwsim::simulator;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;

int main(int argc, char** argv)
{
    // we need
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("output,o", value<string>()->default_value("out.xml"), "the output file.")
        ("oformat,b", value<string>()->default_value("RWTASK"), "The output format, RWTASK, UIBK, Text.")
        ("baseline,l", value<string>(), "The base line experiments (Folder).")
        ("input", value<vector<string> >(), "input Files to compare with baseline (optional).")
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
        return 0;
    }

    using namespace boost::filesystem;

    if(!vm.count("baseline")){
        cout << "\n Error: baseline experiments are required!\n";
        return 10;
    }

    // extract base line grasps
    std::vector<std::string> baselinefiles;
    std::string baseinput = vm["baseline"].as<string>();
    path baseip(baseinput);
    if( is_directory(baseip) ){
        baselinefiles = IOUtil::getFilesInFolder( baseip.string(), false, true);
    } else {
        baselinefiles.push_back( baseip.string() );
    }


    // extract all task files that should be simulated
    std::vector<std::string> infiles;
    const std::vector<std::string> &inputs = vm["input"].as<vector<string> >();
    BOOST_FOREACH(std::string input, inputs){
        path ip(input);
        if( is_directory(ip) ){
            infiles = IOUtil::getFilesInFolder( ip.string(), false, true);
        } else {
            infiles.push_back( ip.string() );
        }
    }

    // resolve output directory
    //path outputfile( vm["output"].as<std::string>() );
    std::string outputdir = vm["output"].as<std::string>();

    std::string outformat = vm["oformat"].as<std::string>();
    int iformat = 0;
    if(outformat=="RWTASK"){ iformat=0;}
    else if(outformat=="UIBK"){iformat=1;}
    else if(outformat=="Text"){iformat=2;}
    else { RW_THROW("Unknown format:" << outformat << ". Please choose one of: RWTASK, UIBK, Text\n"); }

    // first check if we need to do a comparison or just baseline statistics
    if(infiles.size()==0){
        // compile baseline statistics
        int targets = 0, results = 0;
        std::vector<int> testStat(GraspTask::SizeOfStatusArray,0);

        BOOST_FOREACH(std::string ifile, baselinefiles){
            GraspTask::Ptr grasptask = GraspTask::load( ifile );
            // get all stats from grasptask
            BOOST_FOREACH(GraspSubTask& stask, grasptask->getSubTasks()){
                BOOST_FOREACH(GraspTarget& target, stask.getTargets()){
                    targets++;
                    if(target.result==NULL){
                        testStat[GraspTask::UnInitialized]++;
                        continue;
                    }
                    results++;
                    testStat[target.result->testStatus]++;
                }
            }
        }
        std::cout << "\n";
        for(int i=0;i<testStat.size();i++){
            std::cout << GraspTask::toString((GraspTask::TestStatus)i) << "\t";
        }
        std::cout << "\n";
        for(int i=0;i<testStat.size();i++){
            std::cout << testStat[i] << "\t";
        }
        std::cout << std::endl;

    } else {
        // compare baseline with inputfiles
        // here we need to find the files in inputfiles that match the files in baseline files
        // the criteria is that the first part of the filename of input must match that of baseline
        std::vector<std::pair<std::string, std::string> > matches;
        BOOST_FOREACH(std::string inputl, infiles){
            std::string namein = path(inputl).filename().string();
            BOOST_FOREACH(std::string basel, baselinefiles){
                std::string name = path(basel).filename().string();
                if(name == namein.substr(0,name.size())){
                    matches.push_back(make_pair(basel,inputl));
                    break;
                }
            }
        }
        std::cout << "NR of matches: " << matches.size() << std::endl;

        // foreach match we write the test status of each grasp
        typedef std::pair<std::string,std::string> StrPair;
        BOOST_FOREACH(StrPair data, matches){
            GraspTask::Ptr baselinetask = GraspTask::load( data.first );
            GraspTask::Ptr inputtask = GraspTask::load( data.second );

            // load results into two large vectors and compare them, if they are not of the same size then something went wrong

        }
    }

    std::cout << "Done" << std::endl;
    return 0;
}

