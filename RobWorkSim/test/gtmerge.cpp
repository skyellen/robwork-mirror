
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

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

#include "VisGraBGraspTask.hpp"
#include "VisGraBBenchmark.hpp"

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
        ("output,o", value<string>()->default_value("out.xml"), "the output file.")
        ("oformat,b", value<string>()->default_value("RWTASK"), "The output format, RWTASK, UIBK, Text.")
        ("exclude,e", value<std::vector<string> >(), "Exclude grasps based on TestStatus.")
        ("include,i", value<std::vector<string> >(), "Include grasps based on TestStatus. ")
        ("input", value<vector<string> >(), "input Files to merge.")
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
    const std::vector<std::string> &includes = vm["include"].as<vector<string> >();
    BOOST_FOREACH(std::string include, includes){
        if(include=="Success"){ includeMap[GraspTask::Success] = true; }
        else if(include=="ObjectSlipped"){ includeMap[GraspTask::ObjectSlipped] = true; }
        else { RW_THROW("Unsupported include tag!"); }
    }

    // resolve output directory
    path outputfile( vm["output"].as<std::string>() );
    std::string outformat = vm["oformat"].as<std::string>();
    int iformat = 0;
    if(outformat=="RWTASK"){ iformat=0;}
    else if(outformat=="UIBK"){iformat=1;}
    else if(outformat=="Text"){iformat=2;}
    else { RW_THROW("unknown format:" << outformat); }

    GraspTask::Ptr gtask;
    const std::vector<std::string> &inputs = vm["input"].as<vector<string> >();
    int targets = 0, totaltargets = 0;
    std::vector<int> testStat(GraspTask::SizeOfStatusArray,0);
    BOOST_FOREACH(std::string input, inputs){
        path ip(input);
        std::vector<std::string> infiles;
        if( is_directory(ip) ){
            infiles = IOUtil::getFilesInFolder( ip.string(), false, true);
        } else {
            infiles.push_back( ip.string() );
        }

        BOOST_FOREACH(std::string ifile, infiles){
            std::cout << "Processing: " << path(ifile).filename() << " ";
            GraspTask::Ptr grasptask = GraspTask::load( ifile );
            if(gtask==NULL){
                gtask = ownedPtr( new GraspTask() );
                gtask->getRootTask()->getPropertyMap() = grasptask->getRootTask()->getPropertyMap();
            }
            // put all subtasks into gtask
            BOOST_FOREACH(CartesianTask::Ptr stask, grasptask->getRootTask()->getTasks()){
                std::vector<CartesianTarget::Ptr> filteredTargets;
                BOOST_FOREACH(CartesianTarget::Ptr target, stask->getTargets()){
                    int teststatus = target->getPropertyMap().get<int>("TestStatus");
                    if(teststatus<0)
                        teststatus=0;
                    testStat[teststatus]++;
                    totaltargets++;
                    if( includeMap.find(teststatus)!=includeMap.end() ){
                        targets++;
                        filteredTargets.push_back(target);
                    }
                }
                stask->getTargets() = filteredTargets;
                if(filteredTargets.size()>0)
                    gtask->getRootTask()->addTask( stask );
            }
            std::cout << totaltargets << "," << targets << std::endl;
        }
    }
    std::cout << "Statistics: ";
    for(int i=0;i<GraspTask::SizeOfStatusArray;i++)
        std::cout << i << ":" << testStat[i] << " ; ";
    std::cout << std::endl;


    std::cout << "Saving to: " << outputfile.string() << std::endl;
    if(iformat==0){
        GraspTask::saveRWTask(gtask, outputfile.string() );
    } else if(iformat==1){
        GraspTask::saveUIBK(gtask, outputfile.string() );
    } else if(iformat==2){
        GraspTask::saveText(gtask, outputfile.string() );
    }
    std::cout << "Done" << std::endl;
    return 0;
}

