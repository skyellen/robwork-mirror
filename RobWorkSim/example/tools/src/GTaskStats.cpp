
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
#include <boost/numeric/ublas/matrix.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace boost::program_options;

std::vector<std::pair<GraspSubTask*,GraspTarget*> > getTargets(GraspTask::Ptr gtask);
void printConfMatrix(boost::numeric::ublas::bounded_matrix<int, 7, 7>& mat);

int main(int argc, char** argv)
{
    // we need
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("output,o", value<string>()->default_value("out"), "the output folder.")
        ("oformat,b", value<string>()->default_value("RWTASK"), "The output format, RWTASK, UIBK, Text.")
        ("baseline,l", value<string>(), "The base line experiments (Folder).")
        ("silent", value<bool>()->default_value(false), "suppress everything exept results.")
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
    bool silent = vm["silent"].as<bool>();


    path output_dir(vm["output"].as<std::string>());
    if( !is_directory(output_dir) ){
        create_directory(output_dir);
    }

    // extract base line gasps
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
    if(vm.count("input")){
        const std::vector<std::string> &inputs = vm["input"].as<vector<string> >();
        BOOST_FOREACH(std::string input, inputs){
            path ip(input);
            if( is_directory(ip) ){
                infiles = IOUtil::getFilesInFolder( ip.string(), false, true);
            } else {
                infiles.push_back( ip.string() );
            }
        }
    }

    // first check if we need to do a comparison or just baseline statistics
    if(infiles.size()==0){
        // compile baseline statistics
        if(!silent)
            std::cout << "Doing baseline statistics" << std::endl;
        int targets = 0, results = 0;
        std::vector<int> testStat(GraspResult::SizeOfStatusArray,0);

        BOOST_FOREACH(std::string ifile, baselinefiles){
            //std::cout << "Processing: " << ifile << std::endl;
            GraspTask::Ptr grasptask;
            try{
              grasptask = GraspTask::load( ifile );
            } catch(...) {
               //std::cout << "\t\tFailed..." << std::endl;
              continue;
            }
            // get all stats from grasptask
            BOOST_FOREACH(GraspSubTask& stask, grasptask->getSubTasks()){
                BOOST_FOREACH(GraspTarget& target, stask.getTargets()){
                    targets++;
                    if(target.result==NULL){
                        testStat[GraspResult::UnInitialized]++;
                        continue;
                    }
                    results++;
                    testStat[target.result->testStatus]++;
                }
            }
        }
        if(!silent){
            std::cout << "\n";
            for(size_t i=0;i<testStat.size();i++){
                std::cout << GraspTask::toString((GraspTask::TestStatus)i) << "\t";
            }
            std::cout << "\n";
        }
        for(size_t i=0;i<testStat.size();i++){
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
        std::cout << "NR of matches: " << matches.size() << " from (" << baselinefiles.size() << ";" << infiles.size()<< ")" << std::endl;

        using namespace boost::numeric::ublas;

        bounded_matrix<int, 7, 7> confMatTotal = zero_matrix<int>(7,7);

        // foreach match we write the test status of each grasp
        typedef std::pair<std::string,std::string> StrPair;
        BOOST_FOREACH(StrPair data, matches){
            if(!silent){
                std::cout << "processing: \n-" << path(data.first).filename().string();
                std::cout << "\n-" << path(data.second).filename().string() << std::endl;
            }
            GraspTask::Ptr baselinetask = GraspTask::load( data.first );
            GraspTask::Ptr inputtask = GraspTask::load( data.second );

            bounded_matrix<int, 7, 7> confMat = zero_matrix<int>(7,7);

            // load results into two large vectors and compare them, if they are not of the same size then something went wrong
            std::vector<std::pair<GraspSubTask*,GraspTarget*> > baselinetargets = getTargets(baselinetask);
            std::vector<std::pair<GraspSubTask*,GraspTarget*> > inputtargets = getTargets(inputtask);
            if(!silent)
                std::cout << "ListSize: (" << baselinetargets.size() << ";"<< inputtargets.size() << ")" << std::endl;

            size_t minsize = std::min(baselinetargets.size(), inputtargets.size());
            for(size_t i = 0; i<minsize; i++){
                int tstatus1 = baselinetargets[i].second->getResult()->testStatus;
                int tstatus2 = inputtargets[i].second->getResult()->testStatus;
                int stat1=0,stat2=0; // 0 is collision, 1 success, 2

                if(tstatus1==GraspResult::ObjectSlipped || tstatus1==GraspResult::Success ){ stat1 = 0;}
                else if(tstatus1==GraspResult::ObjectMissed){ stat1 = 1;}
                else if(tstatus1==GraspResult::ObjectDropped){ stat1 = 2;}
                else if(tstatus1==GraspResult::CollisionEnvironmentInitially){ stat1 = 3;}
                else if(tstatus1==GraspResult::SimulationFailure){ stat1 = 4;}
                else if(tstatus1==GraspResult::CollisionObjectInitially){ stat1 = 5;}
                else { stat1 = 6;}

                if(tstatus2==GraspResult::ObjectSlipped || tstatus2==GraspResult::Success ){ stat2 = 0;}
                else if(tstatus2==GraspResult::ObjectMissed){ stat2 = 1;}
                else if(tstatus2==GraspResult::ObjectDropped){ stat2 = 2;}
                else if(tstatus2==GraspResult::CollisionEnvironmentInitially){ stat2 = 3;}
                else if(tstatus2==GraspResult::SimulationFailure){ stat2 = 4;}
                else if(tstatus2==GraspResult::CollisionObjectInitially){ stat2 = 5;}
                else { stat2 = 6;}

                confMat(stat1,stat2)++;

                double outcomeQual = -1;

                // now we want to save the false positives and the false negatives in seperate task files
                // so that we can investigate them later on
                GraspResult::Ptr result = baselinetargets[i].second->getResult();
                if(stat1 == 0 && stat2 == 0){
                    // true positive
                    outcomeQual = 0;
                } else if(stat1 == 1 && stat2==1 ) {
                    outcomeQual = 1;
                } else if(stat1 == 0 && stat2==1 ) {
                    outcomeQual = 2;
                } else if(stat1 == 0 && stat2==2 ) {
                    outcomeQual = 3;
                } else if(stat1 == 1 && stat2==0 ) {
                    outcomeQual = 4;
                } else if(stat1 == 2 && stat2==0 ) {
                    outcomeQual = 5;
                } else if(stat1 == 2 && stat2==1 ) {
                    outcomeQual = 6;
                } else if(stat1 == 1 && stat2==2 ) {
                    outcomeQual = 7;
                } else {
                    outcomeQual = 8;
                }

                EAA<> diff( result->objectTtcpTarget.R()* inverse(result->objectTtcpGrasp.R()) );
                Vector3D<> diffp = result->objectTtcpTarget.P() - result->objectTtcpGrasp.P();

                result->qualityAfterLifting = concat(Q(2,outcomeQual, 0.1), result->qualityAfterLifting);
                result->qualityBeforeLifting = concat(Q(2,outcomeQual, 0.1), result->qualityBeforeLifting);
                std::cout << stat1 << "\t" << stat2
                        << "\t" << diffp[0] << "\t" << diffp[1] << "\t" << diffp[2]
                        << "\t" << diff[0] << "\t" << diff[1] << "\t" << diff[2] << "\n";
            }
            confMatTotal = confMatTotal+confMat;
            if(!silent)
                printConfMatrix( confMatTotal);

            // save the baseline in an outputfolder
            std::stringstream sstr;
            path baseline_path(data.first);
            sstr << output_dir.string() << "/" << baseline_path.stem().string() << "_comp.task.xml";
            std::cout << "saving output to: \n\t" << sstr.str() << std::endl;
            GraspTask::saveRWTask( baselinetask, sstr.str() );
        }
        printConfMatrix( confMatTotal);
    }

    //std::cout << "Done" << std::endl;
    return 0;
}

std::string groups[] = {"succ","Fmis","Fdrop","Cenv","SimFail","Cobj","other"};


void printConfMatrix(boost::numeric::ublas::bounded_matrix<int, 7, 7>& mat){
    std::cout << "\n";
    std::cout << "\tsucc\tFmis\tFdrop\tCenv\tSimFail\tCobj\tother\n";
    for(int y=0;y<7;y++){
        std::cout << groups[y] << "\t";
        for(int x=0;x<7;x++){
            std::cout << mat(x,y) << "\t";
        }
        std::cout << "\n";
    }
}

std::vector<std::pair<GraspSubTask*,GraspTarget*> > getTargets(GraspTask::Ptr gtask){
    std::vector<std::pair<GraspSubTask*,GraspTarget*> > res;
    BOOST_FOREACH(GraspSubTask& subtask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& gtarget, subtask.getTargets()){
            res.push_back( make_pair(&subtask,&gtarget) );
        }
    }
    return res;
}

