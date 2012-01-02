
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
        ("dwc,d", value<string>()->default_value(""), "The dynamic workcell.")
        ("exclude,e", value<std::vector<string> >(), "Exclude grasps based on TestStatus.")
        ("include,i", value<std::vector<string> >(), "Include grasps based on TestStatus. ")
        ("input", value<vector<string> >(), "input Files to simulate.")
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
        return 1;
    }

    using namespace boost::filesystem;


    std::map<int,bool> includeMap;
    if(vm.count("include")){
        const std::vector<std::string> &includes = vm["include"].as<vector<string> >();
        BOOST_FOREACH(std::string include, includes){
            if(include=="Success"){ includeMap[GraspTask::Success] = true; }
            else if(include=="ObjectSlipped"){ includeMap[GraspTask::ObjectSlipped] = true; }
            else { RW_THROW("Unsupported include tag!"); }
        }
    } else {
        // include all
        for(int i=0;i<GraspTask::SizeOfStatusArray;i++)
            includeMap[i] = true;
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

    std::string dwc_file = vm["dwc"].as<std::string>();
    if(dwc_file==""){
        std::cout << "Error: No dynamic workcell specified!" << std::endl;
        return 5;
    }

    // load workcell
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(dwc_file);
    State initState = dwc->getWorkcell()->getDefaultState();
    // create GraspTaskSimulator
    GraspTaskSimulator::Ptr graspSim = ownedPtr( new GraspTaskSimulator(dwc) );

    // do the simulation

    int targets = 0, totaltargets = 0;
    std::vector<int> testStat(GraspTask::SizeOfStatusArray,0);

    BOOST_FOREACH(std::string ifile, infiles){


        std::cout << "loading: " << path(ifile).filename() << " ";

        std::stringstream sstr;
        //sstr << outputfile.string() << "_" << totaltargets << "_";
        sstr << outputdir << "/" << path(ifile).filename();
        if(iformat==0){
            sstr << ".task.xml";
        } else if(iformat==1){
            sstr << ".uibk.xml";
        } else if(iformat==2){
            sstr << ".txt";
        }
        if( boost::filesystem::exists( path( sstr.str() ) ) ){
            std::cout << "result already exists!" << std::endl;
            continue;
        }

        GraspTask::Ptr grasptask = GraspTask::load( ifile );
        std::cout << "Starting simulation:" << std::endl;


        // temporarilly change refframe to Object change
        BOOST_FOREACH(GraspSubTask &stask, grasptask->getSubTasks()){
            stask.setRefFrame("object");
            // also remove results
            BOOST_FOREACH(GraspTarget &target, stask.getTargets() ){
                target.result = NULL;
            }
        }

        graspSim->load(grasptask);
        graspSim->startSimulation(initState);

        do{
            TimerUtil::sleepMs(500);
            std::vector<int> stat = graspSim->getStat();
            std::cout << "\r";
            BOOST_FOREACH(int i, stat){ std::cout << i << " "; }
            std::cout << std::flush;
        } while(graspSim->isRunning());

        grasptask = graspSim->getResult();
        // save the result
        totaltargets++;

        std::cout << "Saving to: " << sstr.str() << std::endl;
        if(iformat==0){
            GraspTask::saveRWTask(grasptask, sstr.str() );
        } else if(iformat==1){
            GraspTask::saveUIBK(grasptask, sstr.str() );
        } else if(iformat==2){
            GraspTask::saveText(grasptask, sstr.str() );
        }


    }
    std::cout << "Done" << std::endl;
    return 0;
}

