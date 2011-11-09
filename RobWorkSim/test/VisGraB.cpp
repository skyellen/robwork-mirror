
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
#include <boost/filesystem.hpp>
#include "VisGraBGraspTask.hpp"
#include "VisGraBBenchmark.hpp"

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
bool isResultsCompleted(const std::string& filename);


int main(int argc, char** argv)
{
    // we need
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("benchmark,b", value<string>(), "The benchmark description file.")
        ("outputdir,o", value<string>(), "The temporary directory for storing output and intermidiate files.")
        ("threads,t", value<int>()->default_value(1), "The number of parallel threads to use during simulation.")
        ("rwstudio,r", value<string>()->default_value("RobWorkStudio"), "The RobWorkStudio executable, including path.")
        ("ini-file,i", value<string>(), "The RobWorkStudio ini-file that should be used, including path.")
        ("colfiltering,c", value<int>()->default_value(1), "Filter grasp tasks by checking if they collide with point cloud. (1=enabled; 0=disabled)")
        ("usecache", value<int>()->default_value(1), "Use previous simulated results if available. (1=enabled; 0=disabled)")
        ("experiments,e", value<vector<string> >(), "File containing a set of VisGraB experiments descriptions.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("experiments",-1);


    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

    // write standard welcome, status
    std::cout << "****************************************************************" << std::endl;
    std::cout << "* VisGraB Benchmark executer version 0.1.0" << std::endl;
    std::cout << "* Converts experiments to RobWork simulation task files and \n"
                 "* execute simulation of these in RobWorkStudio" << std::endl;
    std::cout << " " << std::endl;

    if (vm.count("help")) {
        cout << "Usage:\n\n"
                  << "\t" << argv[0] <<" [options] <expFile1> <expFile2> <...> <expFileN> \n"
                  << "\n";

        cout << desc << "\n";
        return 1;
    }

    // resolve output directory
    std::string outputdir = std::string(get_current_dir_name()) + "/output";
    if(vm.count("outputdir")){
        outputdir = vm["outputdir"].as<string>();
    }

    // check if output dir exist, else create dir
    if(!boost::filesystem3::exists(outputdir)){
        if(!boost::filesystem3::create_directory(outputdir)){
            Log::errorLog() << "\nError\nCould not create output directory!\n\n";
            return 4;
        }
    }

    if(vm.count("benchmark")<1){
        cout << "\n<ERROR>\nNo benchmarks specified, please try again and specify a valid VisGraB benchmark manifest.\n"
             << "Use \"VisGraB --help\" for option descriptions.\n\n";
        return 1;
    }

    if (vm.count("experiments")<1) {
        cout << "\nNo experiments specified, please try again and specify a valid VisGraB experiment file.\n"
                << "Use \"VisGraB --help\" for option descriptions.\n\n";
        return 2;
    }

    std::string robworkstudioExe = vm["rwstudio"].as<std::string>();
    if(!boost::filesystem3::exists(robworkstudioExe)){
        Log::errorLog() << "\nError\nCould not find robworkstudio executable: \"" <<robworkstudioExe<< "\" \n\n";
        return 5;
    }

    if (vm.count("experiments")<1) {
        cout << "\nNo experiments specified, please try again and specify a valid VisGraB experiment file.\n"
                << "Use \"VisGraB --help\" for option descriptions.\n\n";
        return 2;
    }

    int colfiltering = vm["colfiltering"].as<int>();
    bool usecache = vm["usecache"].as<int>() != 0;
    std::string robworkstudioIniFile;
    if (vm.count("ini-file")) {
        robworkstudioIniFile = vm["ini-file"].as<std::string>();
    }

    // load the progress file if it exists
    PropertyMap progress;
    std::string progressfile = outputdir + "/" + "progress.xml";
    if(boost::filesystem3::exists(progressfile)){
        progress = XMLPropertyLoader::load( progressfile );
    }

    // load the benchmark xml file
    std::string benchmarkFile = vm["benchmark"].as<std::string>();
    if(!boost::filesystem3::exists(benchmarkFile)){
        Log::errorLog() << "\nError\nCould not find benchmarkFile: \"" <<benchmarkFile<< "\" \n\n";
        return 5;
    }
    VisGraBBenchmark::Ptr benchmark = VisGraBBenchmark::load(benchmarkFile);
    progress.set<std::string>("benchmark", benchmarkFile);

    // ************************************************************************* //
    // this provides location of scene files and stuff
    std::vector<std::pair<std::string,std::string> > rwTaskFiles;
    std::map<std::string,  std::string> rwtaskToSceneID;
    cout << "Generating RobWork task files...\n"
            "RobWork task files are saved under <outputdir> in the format\n\t \"rwtask_<experimentsfilenr>_<experimenttasknr>.task.xml\"\n"
            "Output is also saved to <outputdir> and in the format the format\n\t \"rwtask_<experimentsfilenr>_<experimenttasknr>_result.task.xml\"\n\n";

    cout << "Nr of experiment files to load, convert and execute: " << vm["experiments"].as<vector<string> >().size() << std::endl;
    const std::vector<std::string> &filenames = vm["experiments"].as<vector<string> >();
    int expFileNr = 0;
    BOOST_FOREACH(const std::string& filename, filenames){

        // check that the file(s) exists.
        cout << "Loading: " << filename << std::endl;
        if( !boost::filesystem3::exists( filename ) ){
            Log::errorLog() << "Error: Experiment file: " << StringUtil::quote(filename) << " does not exist!" << std::endl;
            return 3;
        }

        boost::filesystem3::path p(filename);
        if(usecache && progress.has(p.filename().string()) ){
            PropertyMap& fileprogress = progress.get<PropertyMap>(p.filename().string());
            if(fileprogress.has("Complete")){
                std::cout << "Using CACHED RobWork tasks for: "<< p.filename().string() << std::endl;
                std::vector<string> ofiles = fileprogress.get<std::vector<std::string> >("outfiles");
                std::vector<string> rfiles = fileprogress.get<std::vector<std::string> >("resultfiles");
                std::vector<string> sids = fileprogress.get<std::vector<std::string> >("sceneids");
                for(int i=0;i<ofiles.size();i++){
                    rwTaskFiles.push_back(make_pair(ofiles[i],rfiles[i]));
                    rwtaskToSceneID[ ofiles[i] ] = sids[i];
                }
                // we have used the cache to avoid loading the experimentfile
                continue;
            }
        }


        vector<VisGraBGraspTask::Ptr> tasks = VisGraBGraspTask::load(filename);
        cout << "Saving";

        //std::cout << tasks.size() << " experiment sets have been loaded." << std::endl;
        //std::cout << "Saving RobWork task files to: "<< outputdir << std::endl;

        // now save these tasks in RobWork file format
        int taskNr=0;
        std::vector<string> ofiles, rfiles, sids;
        BOOST_FOREACH(VisGraBGraspTask::Ptr vistask, tasks){
            std::cout << "." << std::flush;
            std::stringstream outfile, resultfile;
            outfile << outputdir << "/rwtask_"<< p.filename().string()<< "_" << ++taskNr << ".task.xml";
            resultfile << outputdir << "/rwtask_"<< p.filename().string()<< "_" << ++taskNr << "_result.task.xml";
            GraspTask::saveRWTask(vistask, outfile.str());
            rwTaskFiles.push_back(make_pair(outfile.str(),resultfile.str()));
            std::string sceneID = vistask->getRootTask()->getPropertyMap().get<std::string>("SceneID");
            rwtaskToSceneID[outfile.str()] = sceneID;
            ofiles.push_back(outfile.str() );
            rfiles.push_back(resultfile.str() );
            sids.push_back(sceneID);
        }
        PropertyMap fprogress;
        fprogress.set<std::vector<std::string> >("outfiles", ofiles);
        fprogress.set<std::vector<std::string> >("resultfiles", rfiles);
        fprogress.set<std::vector<std::string> >("sceneids", sids);
        fprogress.set<std::string>("Complete","");
        progress.set<PropertyMap>(p.filename().string(), fprogress);
        std::cout << "\n";
        expFileNr++;
    }
    XMLPropertySaver::save(progress, progressfile);
    using namespace boost::filesystem3;
    // ************************************************************************* //
    // if enabled check if the configurations collide with the point cloud data
    typedef std::pair<std::string,std::string> stringPair;
    if( colfiltering == 1 ){
        cout << "The generated RobWork tasks are filtered for grasps that are in initial contact. (.) generated, (*) Cached\nProcessing" << std::endl;
        int targetsInCollision = 0;
        int targetsTotal = 0;
        BOOST_FOREACH(const stringPair& pair, rwTaskFiles){
            std::string sceneID = rwtaskToSceneID[pair.first];
            // get scene information from benchmark data

            if(usecache && progress.get<PropertyMap>(pair.first, PropertyMap()).has("FilteredComplete")){
                std::cout << "*";
                //std::cout << "Using CACHED collision filtered tasks for: " << path(pair.first).filename().string() << std::endl;
                continue;
            } else {
                std::cout << ".";
                //std::cout << "Generating collision filtered tasks for: " << path(pair.first).filename().string() << std::endl;
            }
            std::cout.flush();

            size_t sid = benchmark->findSceneFromID(sceneID);
            if(sid<0){
                Log::errorLog() << "Error: The sceneID \""<< sceneID << "\" does not exist in the provided benchmark!" << std::endl;
                return 6;
            }
            std::string wcFile = benchmark->getSceneWCData(sid);
            std::string colData = benchmark->getSceneCollisionData(sid);
            std::string runFile = pair.first;

            WorkCell::Ptr wc = WorkCellLoader::load( wcFile );
            Geometry::Ptr colgeom = GeometryFactory::load(colData,true);
            CollisionStrategy::Ptr strategy = ProximityStrategyFactory::makeDefaultCollisionStrategy();
            strategy->addModel(wc->getWorldFrame(), colgeom);
            GraspTask::Ptr gtask = GraspTask::load( runFile );
            State state = wc->getDefaultState();
            std::string gripperId = gtask->getRootTask()->getPropertyMap().get<std::string>("Gripper");
            std::string tcpId = gtask->getRootTask()->getPropertyMap().get<std::string>("TCP");
            Device::Ptr gripper = wc->findDevice(gripperId);


            MovableFrame *mbase = dynamic_cast<MovableFrame*> (gripper->getBase());
            Frame* tcp = wc->findFrame(tcpId);
            Transform3D<> baseTtcp = Kinematics::frameTframe(mbase,tcp,state);
            // test collision between gripper and world
            std::vector<Frame*> gframes = Kinematics::findAllFrames( gripper->getBase() );

            ProximitySetup setup;
            setup.setUseIncludeAll(false);
            BOOST_FOREACH(Frame* f, gframes){
                setup.addProximitySetupRule( ProximitySetupRule::makeInclude(f->getName(), wc->getWorldFrame()->getName()) );
            }
            CollisionDetector colDet(wc,strategy,ownedPtr(new BasicFilterStrategy(wc,setup)));


            // get the gripper
            //std::cout << "NR OF TASKS: " <<  gtask->getRootTask()->getTasks().size() << std::endl;
            BOOST_FOREACH(CartesianTask::Ptr task, gtask->getRootTask()->getTasks() ){

                std::vector<CartesianTarget::Ptr> validtargets;
                Q openQ = task->getPropertyMap().get<Q>("OpenQ", Q(gripper->getDOF(),0.0));
                gripper->setQ(openQ, state);
                BOOST_FOREACH(CartesianTarget::Ptr target, task->getTargets()){
                    // set gripper pose
                    Transform3D<> wTbase = target->get() * inverse(baseTtcp);
                    mbase->setTransform( wTbase , state);
                    targetsTotal++;
                    if( colDet.inCollision(state, NULL, true) ){
                        //validtargets.push_back(target);
                        target->getPropertyMap().set<int>("TestStatus",GraspTask::CollisionFiltered);
                        targetsInCollision++;
                        std::cout << "\r Total targets and colliding targets: [ " << targetsTotal << " , " << targetsInCollision << " , " << (targetsInCollision*100.0/targetsTotal) << "% ] ";
                    }
                }
                //task->getTargets() = validtargets;
            }
            GraspTask::saveRWTask( gtask, runFile );
            progress.get<PropertyMap>(pair.first, PropertyMap()).set<std::string>("FilteredComplete","");
        }
        cout << std::endl;
        XMLPropertySaver::save(progress, progressfile);
    } else {
        cout << "Collision pre-filtering disabled!" << std::endl;
    }


    // ************************************************************************* //
    // now do all the simulation, that is
    cout << "Starting simulations..." << std::endl;
    BOOST_FOREACH(const stringPair& pair, rwTaskFiles){
        std::string sceneID = rwtaskToSceneID[pair.first];
        // get scene information from benchmark data

        size_t sid = benchmark->findSceneFromID(sceneID);
        if(sid<0){
            Log::errorLog() << "Error: The sceneID \""<< sceneID << "\" does not exist in the provided benchmark!" << std::endl;
            return 6;
        }
        std::string dwcFile = benchmark->getSceneDWCData(sid);

        // execute RobWorkStudio
        std::stringstream cmd2;
        //cout << "Starting simulation batch: \n\t" << pair.first << std::endl;
        //cout << "* " << std::endl;
        std::string runFile = pair.first;
        std::string resultFile = pair.second;

        if( boost::filesystem3::exists( resultFile )){
            if(usecache){
                runFile = resultFile;
            } else {
                // make sure to delete the result file before starting simulation
                boost::filesystem3::remove( resultFile );
            }
        }

#ifdef WIN32
        cmd2 << "start "
             << " \""<<runFile.substr(runFile.size()-40,40).c_str()<<"\""
             << " /WAIT"
             << robworkstudioExe << " -PAuto=True -PNoSave=True"
             << " -PDWC=" << dwcFile
             << " -PTaskTestFile=" << runFile
             << " -PTaskTestOutFile=" << resultFile;
        if(robworkstudioIniFile!="")
             cmd2 << " --ini-file=" << robworkstudioIniFile;
#else
        cmd2 << robworkstudioExe << " -PAuto=True -PNoSave=True"
             << " -PDWC=" << dwcFile
             << " -PTaskTestFile=" << runFile
             << " -PTaskTestOutFile=" << resultFile;
        if(robworkstudioIniFile!="")
             cmd2 << " --ini-file=" << robworkstudioIniFile;
        cmd2 << " > log.txt 2> errlog.txt ";
#endif
        //cout << "EXECUTING: \n" << cmd2.str() << "\n" << endl;
        std::cout << "Processing:" << path(runFile).filename().string() << ", " << std::flush;
        boost::filesystem3::path p(robworkstudioExe);
        if( runFile == resultFile && isResultsCompleted(resultFile) ){
            cout << "USING_CACHE, COMPLETE" << std::endl;
            TimerUtil::sleepMs( 100 );
            continue;
        }
        TimerUtil::sleepMs( 1000 );
        std::string cdCmd("cd ");
        cdCmd.append( p.parent_path().c_str() );
        cdCmd.append( ";");
        cdCmd.append( cmd2.str());
        system(cdCmd.c_str());
        //int ret = system(cmd2.str().c_str());
        std::cout << "Done, "<< std::flush;
        // now test if all targets have been processed
        TimerUtil::sleepMs( 1000 );
        bool isComplete = isResultsCompleted(resultFile);

        if(!isComplete){
            std::cout << "INCOMPLETE" << std::endl;
        } else {
            std::cout << "COMPLETE" << std::endl;
        }
    }


    // last we convert the result into VisGraB format.
    return (0);
}

bool isResultsCompleted(const std::string& filename){
    bool isIncomplete = false;
    GraspTask::Ptr gtask;
    try {
        GraspTask::Ptr gtask = GraspTask::load( filename );
        BOOST_FOREACH(CartesianTask::Ptr task, gtask->getRootTask()->getTasks() ){
            BOOST_FOREACH(CartesianTarget::Ptr target, task->getTargets()){
                int teststatus = target->getPropertyMap().get<int>("TestStatus");
                if(teststatus==GraspTask::UnInitialized){
                    isIncomplete = true;
                }
            }
        }

    } catch (...){
        isIncomplete = true;
    }
    return !isIncomplete;
}

/*

<visgrab>
 <results>
  <experiments url=""/> <!-- this should point to the file used for experiments -->
  <result grasp="1">
   <!-- we need the executed grasp since this could have been generated as a result of the inverse kinematics -->
   <grasp>
    <pose><pos>0 0 0 </pos> <quaternion>0 0 0 0</quaternion></pose>
    <qopen> 0 0 0 0 0 0 0</qopen>
    <qclosed> 0 0 0 0 0 0 0</qclosed>
    <maxtau> 0 0 0 0 0 0 0</maxtau>
   <grasp>

   <failure type="" /> <!-- the failure types that we allready have -->
   |
   <success> <!-- the type is either lifted or slipped -->
    <quality>0 0 0 0 0</quality> <!-- i imagine slipquality and unit wrench quliaty, but we can put all of them inthere -->
    <qgrasp>0 0 0 0 0 0 0 </qgrasp> <!-- the grasp configuration after grasp -->
    <qlift>0 0 0 0 0 0 0 </qlift> <!-- the grasp configuration after lift -->

    <object2tcp state="AfterGrasp"> <pos>0 0 0 </pos> <quaternion>0 0 0 0</quaternion> </object2tcp>
    <object2tcp state="AfterLift"> <pos>0 0 0 </pos> <quaternion>0 0 0 0</quaternion> </object2tcp>
    <contacts>
     <contact> <force>0</force> <pos>0 0 0</pos> <normal>0 0 0</normal> </contact>
     <contact> <force>0</force> <pos>0 0 0</pos> <normal>0 0 0</normal> </contact>
     <contact> <force>0</force> <pos>0 0 0</pos> <normal>0 0 0</normal> </contact>
     ...
    </contacts>
   </success>
  </result>

  <result grasp"2">
  ...
  </result>
  ...

 </results>
</visgrab>

*/

//CarteseanTask::Ptr readExperiments(const std::string& filename){
    // parse the VisGraB experiments format and translate it into the RobWork Task format


//}



