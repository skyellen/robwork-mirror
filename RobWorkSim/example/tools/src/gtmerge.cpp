
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
        ("output,o", value<string>()->default_value("out.xml"), "the output file.")
        ("oformat,b", value<string>()->default_value("RWTASK"), "The output format, RWTASK, UIBK, Text.")
        ("exclude,e", value<std::vector<string> >(), "Exclude grasps based on TestStatus.")
        ("include,i", value<std::vector<string> >(), "Include grasps based on TestStatus. ")
        ("input", value<vector<string> >(), "input Files to merge.")
        ("batchsize", value<int>()->default_value(5000), "The max nr grasp of output, files will be split into multiple files.")
        ("useGraspTarget",value<bool>()->default_value(false),"Enable to copy ObjTTcpTarget to target.pose")
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

    int batchsize = vm["batchsize"].as<int>();
    using namespace boost::filesystem;

    bool useGraspTarget = vm["useGraspTarget"].as<bool>();
    std::map<int,bool> includeMap;
    if(vm.count("include")){
        const std::vector<std::string> &includes = vm["include"].as<vector<string> >();
        BOOST_FOREACH(std::string include, includes){
            if(include=="Success"){ includeMap[GraspResult::Success] = true; }
            else if(include=="ObjectSlipped"){ includeMap[GraspResult::ObjectSlipped] = true; }
            else { RW_THROW("Unsupported include tag!"); }
        }
    } else {
        // include all
        for(int i=0;i<GraspResult::SizeOfStatusArray;i++)
            includeMap[i] = true;
    }

    if(vm.count("exclude")){
        const std::vector<std::string> &excludes = vm["exclude"].as<vector<string> >();
        BOOST_FOREACH(std::string exclude, excludes){
            if(exclude=="Success"){ includeMap[GraspResult::Success] = false; }
            else if(exclude=="ObjectSlipped"){ includeMap[GraspResult::ObjectSlipped] = false; }
            else if(exclude=="Collision"){
                includeMap[GraspResult::CollisionFiltered] = false;
                includeMap[GraspResult::CollisionObjectInitially] = false;
                includeMap[GraspResult::CollisionEnvironmentInitially] = false;
                includeMap[GraspResult::CollisionInitially] = false;
            }

            else { RW_THROW("Unsupported exclude tag!"); }
        }
    }

    // resolve output directory
    path outputfile( vm["output"].as<std::string>() );
    std::string outformat = vm["oformat"].as<std::string>();
    int iformat = 0;
    if(outformat=="RWTASK"){ iformat=0;}
    else if(outformat=="UIBK"){iformat=1;}
    else if(outformat=="Text"){iformat=2;}
    else { RW_THROW("Unknown format:" << outformat << ". Please choose one of: RWTASK, UIBK, Text\n"); }

    GraspTask::Ptr gtask;
    const std::vector<std::string> &inputs = vm["input"].as<vector<string> >();
    int targets = 0, totaltargets = 0, localNrTargets=0,nrBatches=0;
    std::vector<int> testStat(GraspResult::SizeOfStatusArray,0);
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
                gtask->setGripperID( grasptask->getGripperID() );
                gtask->setTCPID( grasptask->getTCPID() );
                gtask->setGraspControllerID( grasptask->getGraspControllerID() );
                //gtask->getRootTask()->getPropertyMap() = grasptask->getRootTask()->getPropertyMap();
            }
            // put all subtasks into gtask

            BOOST_FOREACH(GraspSubTask &stask, grasptask->getSubTasks()){
                std::vector<GraspTarget> filteredTargets;
                BOOST_FOREACH(GraspTarget &target, stask.targets){
                    if(target.result==NULL)
                        continue;
                    int teststatus = target.result->testStatus;
                    if(teststatus<0)
                        teststatus=0;
                    testStat[teststatus]++;
                    totaltargets++;
                    if( useGraspTarget ){
                        target.pose = target.result->objectTtcpGrasp;
                    }

                    if( includeMap[teststatus] ){
                        targets++;
                        localNrTargets++;
                        filteredTargets.push_back(target);
                    }
                }
                stask.targets = filteredTargets;
                if(filteredTargets.size()>0)
                    gtask->getSubTasks().push_back( stask );
            }
            if(localNrTargets>batchsize){
                localNrTargets=0;
                //TODO save gtask
                std::stringstream sstr;
                if(iformat==0){
                    sstr << outputfile.string() << "_" << nrBatches << ".rwtask.xml";
                    GraspTask::saveRWTask(gtask, sstr.str());
                } else if(iformat==1){
                    sstr << outputfile.string() << "_" << nrBatches << ".uibk.xml";
                    GraspTask::saveUIBK(gtask, sstr.str() );
                } else if(iformat==2){
                    sstr << outputfile.string() << "_" << nrBatches << ".txt";
                    GraspTask::saveText(gtask, sstr.str() );
                }
                nrBatches++;

            }
            std::cout << totaltargets << "," << targets << std::endl;
        }
    }
    std::cout << "Statistics: ";
    for(int i=0;i<GraspResult::SizeOfStatusArray;i++)
        std::cout << i << ":" << testStat[i] << " ; ";
    std::cout << std::endl;


    std::cout << "Saving to: " << outputfile.string() << std::endl;
    if(iformat==0){
        GraspTask::saveRWTask(gtask, outputfile.string()+".rwtask.xml" );
    } else if(iformat==1){
        GraspTask::saveUIBK(gtask, outputfile.string()+".uibk.xml" );
    } else if(iformat==2){
        GraspTask::saveText(gtask, outputfile.string()+".txt" );
    }
    std::cout << "Done" << std::endl;
    return 0;
}

