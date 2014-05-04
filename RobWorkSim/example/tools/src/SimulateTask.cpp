
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
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>

#include <rw/RobWork.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

#include <rw/models/Models.hpp>

#include <rwsimlibs/ode/ODESimulator.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rwlibs::task;
using namespace rwsim::simulator;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace boost::program_options;

void addPertubations(GraspTask::Ptr grasptask, double sigma_p, double sigma_a, int pertubationsPerTarget);

std::vector<GraspTask::Ptr> splitTask(GraspTask::Ptr grasptask, int split){
    int count = 0;
    std::vector<GraspTask::Ptr> gtasks = std::vector<GraspTask::Ptr>(1, grasptask->clone() );
    BOOST_FOREACH(GraspSubTask &stask, grasptask->getSubTasks()){
        // also remove results
        GraspSubTask nstask = stask.clone();
        gtasks.back()->addSubTask( nstask );

        BOOST_FOREACH(GraspTarget &target, stask.getTargets() ){
            gtasks.back()->getSubTasks().back().addTarget( target );
            count++;
            if(count>=split){
                count=0;
                // create new grasp task
                gtasks.push_back( grasptask->clone() );
                GraspSubTask nstask = stask.clone();
                gtasks.back()->addSubTask( nstask );
            }
        }
    }
    return gtasks;
}


int main(int argc, char** argv)
{
    rw::RobWork::getInstance()->initialize();

    Math::seed(time(NULL));
    srand ( time(NULL) );
    Log::log().setDisable(Log::DebugMask);
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
        ("align", value<bool>()->default_value(false), "Use aligned grasps from input.")
        ("perturbe", value<bool>()->default_value(false), "Add small random pertubations to all targets.")
        ("pertubations", value<int>()->default_value(1), "Number of pertubations to perform on each target.")
        ("sigma_a", value<double>()->default_value(2), "Standard deviation in of angle in Degree.")
        ("sigma_p", value<double>()->default_value(0.003), "Standard deviation of position in meters.")
        ("output-statepath", value<bool>()->default_value(false), "State path for visually checking cutting trajectory.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);

    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);
    bool useAlignedGrasp = vm["align"].as<bool>();
    // write standard welcome, status
    if (vm.count("help")) {
        cout << "Usage:\n\n"
                  << "\t" << argv[0] <<" [options] -o<outfile> <expFile1> <expFile2> <...> <expFileN> \n"
                  << "\n";
        cout << desc << "\n";
        return 1;
    }

    using namespace boost::filesystem;

    double sigma_p = vm["sigma_p"].as<double>();
    double sigma_a = vm["sigma_a"].as<double>()*Deg2Rad;
    int pertubationsPerTarget = vm["pertubations"].as<int>();

    bool outputState = vm["output-statepath"].as<bool>();
    TimedStatePath statep;

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
    path outputfile( vm["output"].as<std::string>() );
    //std::string outputdir = vm["output"].as<std::string>();

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
    WorkCell::Ptr wc = dwc->getWorkcell();
    std::cout << "WorkCell: nr frames: " << Models::findAllFrames(*wc).size() << std::endl;

    // create GraspTaskSimulator
    GraspTaskSimulator::Ptr graspSim = ownedPtr( new GraspTaskSimulator(dwc, 1) );

    // do the simulation
    //Unused: int targets = 0;
    int totaltargets = 0;
    std::vector<int> testStat(GraspTask::SizeOfStatusArray,0);
    bool perturbe = vm["perturbe"].as<bool>();
    BOOST_FOREACH(std::string ifile, infiles){
        std::cout << "loading: " << path(ifile).filename() << " ";

        std::stringstream sstr;
        //sstr << outputfile.string() << "_" << totaltargets << "_";
        if(perturbe){
            sstr << outputfile.string() << "_" << sigma_p << "_" << sigma_a << "_" << pertubationsPerTarget;
        } else {
            sstr << outputfile.string();
        }
        if( boost::filesystem::exists( path( sstr.str() ) ) ){
            std::cout << "result already exists!" << std::endl;
            continue;
        }

        GraspTask::Ptr grasptask = GraspTask::load( ifile );
        std::cout << "Starting simulation:" << std::endl;

        // temporarilly change refframe to Object change
        BOOST_FOREACH(GraspSubTask &stask, grasptask->getSubTasks()){
            // also remove results
            BOOST_FOREACH(GraspTarget &target, stask.getTargets() ){
                if(target.result!=NULL){
                    if(useAlignedGrasp){
                        if( target.result->testStatus==GraspTask::Success || target.result->testStatus==GraspTask::ObjectSlipped){
                            target.pose = target.result->objectTtcpLift;
                        }
                    }
                }
                target.result = NULL;
            }
        }
        std::vector<GraspTask::Ptr> tasks;
        if(perturbe){
            addPertubations(grasptask, sigma_p, sigma_a, pertubationsPerTarget );
            int nroftarg = 6000/(pertubationsPerTarget+1);
            tasks = splitTask(grasptask,nroftarg*(pertubationsPerTarget+1));
        } else {
            tasks.push_back(grasptask);
        }

        for(std::size_t i=0;i<tasks.size();i++){
            std::stringstream outputfile;
            if(iformat==0){
                outputfile << sstr.str()+"_"+boost::lexical_cast<std::string>(i)+".task.xml";
            } else if(iformat==1){
                outputfile <<  sstr.str()+"_"+boost::lexical_cast<std::string>(i)+".uibk.xml";
            } else if(iformat==2){
                outputfile <<  sstr.str()+"_"+boost::lexical_cast<std::string>(i)+".txt";
            }
            if( boost::filesystem::exists( path( outputfile.str() ) ) ){
                std::cout << "result already exists!\n\t" << outputfile.str() << std::endl;
                totaltargets++;
                continue;
            }

            std::cout << graspSim->getStatDescription() << std::endl;

            std::cout << std::endl;

            graspSim->load(tasks[i]);
            graspSim->startSimulation(initState);
            if(outputState){
                statep.push_back(TimedState(0,initState));
            }
            for(std::size_t j=0;j<graspSim->getStat().size(); j++){ std::cout << j << "\t"; }
            std::cout<< std::endl;
            TimerUtil::sleepMs(2000);
            do{
                if(outputState){
                    statep.push_back(TimedState(statep.size()*0.01,graspSim->getSimulator()->getState()));
                }
                TimerUtil::sleepMs(100);
                std::vector<int> stat = graspSim->getStat();
                std::cout << "\r";
                BOOST_FOREACH(int i, stat){ std::cout << i << "\t"; }
                std::cout << std::flush;
            } while(graspSim->isRunning());

            grasptask = graspSim->getResult();
            // save the result
            totaltargets++;

            // store timed state
            if(outputState){
                std::cout << "Statepath: " << statep.size() << std::endl;
                PathLoader::storeTimedStatePath(*dwc->getWorkcell(),statep, sstr.str()+".rwplay");
            }
            std::cout << "Saving to: " << sstr.str() << std::endl;
            if(iformat==0){
                GraspTask::saveRWTask(grasptask, sstr.str()+"_"+boost::lexical_cast<std::string>(i)+".task.xml" );
            } else if(iformat==1){
                GraspTask::saveUIBK(grasptask, sstr.str()+"_"+boost::lexical_cast<std::string>(i)+".uibk.xml" );
            } else if(iformat==2){
                GraspTask::saveText(grasptask, sstr.str()+"_"+boost::lexical_cast<std::string>(i)+".txt" );
            }
        }
    }
    std::cout << "Done" << std::endl;
    return 0;
}

void addPertubations(GraspTask::Ptr grasptask, double sigma_p, double sigma_a, int pertubationsPerTarget){
    //double sigma_p = 0.005;
    //double sigma_a = 15*Deg2Rad;
    //int pertubationsPerTarget = 100;
    //Unused: int count = 0;
    // temporarilly change refframe to Object change
    BOOST_FOREACH(GraspSubTask &stask, grasptask->getSubTasks()){
        std::vector<GraspTarget> ntargets;
        // also remove results

        BOOST_FOREACH(GraspTarget &target, stask.getTargets() ){
            ntargets.push_back(target.pose);
            for(int i=0;i<pertubationsPerTarget;i++){
                Vector3D<> pos(Math::ranNormalDist(0,sigma_p), Math::ranNormalDist(0,sigma_p), Math::ranNormalDist(0,sigma_p));
                // we can do this only for small sigmas (approximation)
                EAA<> rot(Math::ranNormalDist(0,sigma_a), Math::ranNormalDist(0,sigma_a), Math::ranNormalDist(0,sigma_a));

                // TODO: we should truncate at 2*sigma sooo

                Transform3D<> ntarget = target.pose*Transform3D<>(pos, rot);
                ntargets.push_back(ntarget);
            }
        }
        stask.getTargets() = ntargets;
    }
}


