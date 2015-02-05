
#include <rw/rw.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/lexical_cast.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/geometry/IntersectUtil.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>

#include "CutActionParam.hpp"

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace boost::program_options;
using namespace std;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::task;


std::vector<int> parseSuccess(std::string file);

int main(int argc, char** argv){
    // we need
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("knife,k", value<string>(), "the knife name.")
        ("object,o", value<string>(), "the object name.")
        ("output-dir", value<string>()->default_value("out"), "The output directory.")
        ("output-geom", value<bool>()->default_value(false), "The geometries of cutted object will be saved.")
        ("output-statepath", value<bool>()->default_value(false), "State path for visually checking cutting trajectory.")
        ("debug", value<bool>()->default_value(false), "Debug stuff.")
        ("wc,w", value<string>(), "the workcell file.")
        ("cutdb", value<string>(), "cutting database file (currently the cutting action param file).")
        ("graspdb", value<string>(), "The grasp database file.")
        ("gripper", value<string>(), "the gripper.")
        ("gripper-tcp", value<string>()->default_value("GRIPPER_TCP"), "the gripper tcp.")
        ("gripper-base", value<string>()->default_value("GRIPPER_MOVER"), "the frame that moves the gripper.")

    ;
    positional_options_description optionDesc;
    optionDesc.add("cutdb",-1);
    variables_map vm;
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

    if (vm.count("help")) {
        cout << "Usage:\n\n"
                  << "\t" << argv[0] <<" [options] -w<workcellFile> -o<objectName> -k<knifeName> <actionInput> \n"
                  << "\n";
        cout << desc << "\n";
        return 1;
    }

    // extract filenames
    std::string wcFile = vm["wc"].as<string>();
    std::string knifeName = vm["knife"].as<string>();
    std::string objName = vm["object"].as<string>();

    std::string graspdb = vm["graspdb"].as<string>();

    string out_dir = vm["output-dir"].as<string>();
    if(!boost::filesystem::exists(out_dir)){
    	boost::filesystem::create_directory(out_dir);
    }

    // parse gripper stuff
    std::string gripper_name = vm["gripper"].as<string>();
    std::string gripper_tcp_name = vm["gripper-tcp"].as<string>();
    std::string gripper_base_name = vm["gripper-base"].as<string>();
    std::string cutdb_name = vm["cutdb"].as<string>();
    // Unused variables:
    //bool output_statepath = vm["output-statepath"].as<bool>();
    //bool output_geom = vm["output-geom"].as<bool>();
    //bool debug = vm["debug"].as<bool>();

    // load grasp database
    Log::infoLog() << "Loading grasp database: " << graspdb << std::endl;
    GraspTask::Ptr gtask = GraspTask::load( graspdb );

    std::vector<CutActionParam> cutDB = CutActionParam::load(cutdb_name);
    // we generate one large cutting param file and simulate it

    std::stringstream cut_actions_output;
    cut_actions_output << out_dir<<"/all_cutting_actions.txt" ;
    Log::infoLog() << "Generating action parameters in file: \n\t" << cut_actions_output.str() << std::endl;
    // foreach grasp suggestion in gtask calculate its quality for cutting
    std::vector<std::pair<GraspSubTask*,GraspTarget*> > targets = gtask->getAllTargets();

    // now perform the cutting simulation
    std::stringstream cmd;
    cmd     << "./cut "
             <<  " -w" << wcFile
             << " --knife="<< knifeName
             << " --object="<< objName
             << " --input="<<cut_actions_output.str()
             << " --output-dir="<< out_dir
             << " --output-geom=false "
             << " --output-statepath=true"
             << " --gripper="<< gripper_name
             << " --gripper-tcp="<< gripper_tcp_name
             << " --gripper-base="<< gripper_base_name
                ;

    double sigma_p = 0.02;
    double sigma_a = 10*Deg2Rad;
    std::vector<CutActionParam> cut_experiments;

    for(std::size_t i=0;i<targets.size();i++){
        //Unused: GraspSubTask *subtask = targets[i].first;
        GraspTarget *target = targets[i].second;
        for(std::size_t j=0;j<cutDB.size();j++){
            CutActionParam base = cutDB[j];

            if(target->result!=NULL && target->result->testStatus==GraspResult::Success){
                CutActionParam exp = base;
                exp.gripperQ = target->result->gripperConfigurationGrasp;
                exp.posGripper = target->result->objectTtcpGrasp.P();
                exp.rpyGripper = RPY<>( target->result->objectTtcpGrasp.R() );

                // to do simulation we prepare a parameter file for the cut script
                // and prepare the console parameters for the cutting

                Vector3D<> pos(Math::ranNormalDist(0,sigma_p), Math::ranNormalDist(0,sigma_p), Math::ranNormalDist(0,sigma_p));
                // we can do this only for small sigmas (approximation)
                EAA<> rot(Math::ranNormalDist(0,sigma_a), Math::ranNormalDist(0,sigma_a), Math::ranNormalDist(0,sigma_a));

                Transform3D<> ntarget = Transform3D<>(base.posObj, base.rpyObj); //*Transform3D<>(pos, rot);
                exp.posObj = ntarget.P();
                exp.rpyObj = RPY<>( ntarget.R() );
                // set gripper object transform and grasp
                cut_experiments.push_back(exp);
            }
        }
    }
    Log::infoLog() << "Generated " << cut_experiments.size() << " experiments!" << std::endl;
    /// simulate all experiments
    CutActionParam::save(cut_experiments, cut_actions_output.str());
    std::cout << "Simulating cutting of grasp: " << std::endl;
    //int ret = system(cmd.str().c_str());
    system(cmd.str().c_str());
    // read back the results
    std::vector<int> successes = parseSuccess(out_dir + "/resultCompact.txt");

    int j=0;
    for(std::size_t i=0;i<targets.size();i++){
        //Unused: GraspSubTask *subtask = targets[i].first;
        GraspTarget *target = targets[i].second;
        if(target->result!=NULL && target->result->testStatus==GraspResult::Success){
            // compute the successRate from the next cutDB.size() successes
            int succCnt=0;
            for(std::size_t n=0;n<cutDB.size();n++){
                if(successes[j]!=0)
                    succCnt++;
                j++;
            }
            double succRatio = (succCnt*1.0)/cutDB.size();
            target->result->qualityBeforeLifting = concat(Q(1,succRatio), target->result->qualityBeforeLifting);
            target->result->qualityAfterLifting = concat(Q(1,succRatio), target->result->qualityAfterLifting);
        }
    }

    GraspTask::saveRWTask( gtask, out_dir + "/graspDB-updated.rwtask.xml");

/*

./cut  --wc=/media/DATA/Data/xperience_demo/cutting-robotiq-3-db-scene/cutting_obj_scene.wc.xml
       --gripper=Robotiq-3
       --gripper-tcp=Robotiq-3.GTCP1
       --gripper-base=Robotiq-3.Base
       --knife=Knife-a
       --object=Banana
       --input=params_test.txt
       --debug=0
       --output-statepath=1

 */



    return 1;
}
std::vector<int> parseSuccess(std::string file){
    std::vector<int> successes;
        std::fstream fstr;
        fstr.open(file.c_str());
        std::string line;
        int totalCnt = 0, succCnt=0;
        while (!fstr.eof()) {
            getline(fstr, line);
            if (line[0] == '#') continue;

            std::stringstream sstr(line);
            int id, success;
            sstr >> id >> success ;
            totalCnt++;
            if(success)
                succCnt++;
            successes.push_back(success);
       }
       return successes;
}
