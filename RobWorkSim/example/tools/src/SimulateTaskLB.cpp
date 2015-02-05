
#include <iostream>
#include <vector>
#include <string>

#include <rw/rw.hpp>
#include <rw/common/LogFileWriter.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rw::loaders;
using namespace rwlibs::task;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace rwsim::simulator;
using namespace boost::program_options;

GraspTask::Ptr generateTasks(int nrTasks, DynamicWorkCell::Ptr dwc, string objectName, string type);


int main(int argc, char** argv)
{
    // we need
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("output,o", value<string>()->default_value("out.xml"), "the output file.")
        ("oformat,b", value<string>()->default_value("RWTASK"), "The output format, RWTASK, UIBK, Text.")
        ("playbackprefix", value<string>()->default_value(""), "set to store a playback file for each target (subtasks and grasps are distinguished by ID in task file).")
        ("dwc,d", value<string>()->default_value(""), "The dynamic workcell.")
        ("object", value<string>()->default_value(""), "if gentask enabled then object need be defined.")
        ("gripper", value<string>()->default_value(""), "if gentask enabled then gripper need be defined.")
        ("gentask", value<bool>()->default_value(false), "if gentask enabled then gripper need be defined.")
        ("exclude,e", value<std::vector<string> >(), "Exclude grasps based on TestStatus.")
        ("include,i", value<std::vector<string> >(), "Include grasps based on TestStatus. ")
        ("input", value<vector<string> >(), "input Files to simulate.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);

    // initialize RobWork log. We put debug into rwdebug.log and warning into rwwarn.log
    Log::log().setWriter(Log::Debug, ownedPtr( new LogFileWriter("rwdebug.log") ) );
    Log::log().setWriter(Log::Warning, ownedPtr( new LogFileWriter("rwwarn.log") ) );

    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

    rw::math::Math::seed( TimerUtil::currentTimeMs() );
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
    std::vector<GraspTask::TestStatus> taskincludefilter;
    if(vm.count("include")){
        const std::vector<std::string> &includes = vm["include"].as<vector<string> >();
        BOOST_FOREACH(std::string include, includes){
            if(include=="Success"){
                includeMap[GraspResult::Success] = true;
                taskincludefilter.push_back(GraspResult::Success);
            }
            else if(include=="ObjectSlipped"){
                includeMap[GraspResult::ObjectSlipped] = true;
                taskincludefilter.push_back(GraspResult::ObjectSlipped);
            }
            else { RW_THROW("Unsupported include tag!"); }
        }
    } else {
        // include all
        for(int i=0;i<GraspResult::SizeOfStatusArray;i++){
            taskincludefilter.push_back( (GraspTask::TestStatus)i );
            includeMap[i] = true;
        }
    }

    // extract all task files that should be simulated
    std::vector<std::string> infiles;
    if(vm.count("input") ){
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
    //

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
    GraspTaskSimulator::Ptr graspSim = ownedPtr( new GraspTaskSimulator(dwc, 1) );

    int nrTasks = 5000, totalNrTasks = 0, taskIndex=0;
    if( vm["gentask"].as<bool>() ){
        string objectName = vm["object"].as<string>();
        string type = vm["gripper"].as<string>();
        int offset = Math::ranI(0,1000) * 20;
        while(1){
            totalNrTasks += nrTasks;
            taskIndex += 1;

            std::stringstream sstr;
            //sstr << outputfile.string() << "_" << totaltargets << "_";
            sstr << outputdir << "/task_" << offset + taskIndex;
            if(iformat==0){
                sstr << ".task.xml";
            } else if(iformat==1){
                sstr << ".uibk.xml";
            } else if(iformat==2){
                sstr << ".txt";
            }
            if(boost::filesystem::exists(sstr.str())){
                std::cout << "File allready simulated... skipping" << std::endl;
                if(totalNrTasks>=100000){
                    return 0;
                }
                continue;
            }


            GraspTask::Ptr grasptask = generateTasks(nrTasks, dwc, objectName, type);

            graspSim->load(grasptask);
            graspSim->startSimulation(initState);
            TimerUtil::sleepMs(2000);
            do{
                TimerUtil::sleepMs(500);
                std::vector<int> stat = graspSim->getStat();
                std::cout << "\r";
                BOOST_FOREACH(int i, stat){ std::cout << i << " "; }
                std::cout << std::flush;
            } while(graspSim->isRunning());

            grasptask = graspSim->getResult();
            // save the result
            grasptask->filterTasks( taskincludefilter );

            //sstr << outputfile.string() << "_" << totaltargets << "_";
            std::cout << "Saving to: " << sstr.str() << std::endl;
            if(iformat==0){
                GraspTask::saveRWTask(grasptask, sstr.str() );
            } else if(iformat==1){
                GraspTask::saveUIBK(grasptask, sstr.str() );
            } else if(iformat==2){
                GraspTask::saveText(grasptask, sstr.str() );
            }


            if(totalNrTasks>=100000){
                return 0;
            }
        }
    }

    // do the simulation
    //Unused: int targets = 0;
    int totaltargets = 0;
    std::vector<int> testStat(GraspResult::SizeOfStatusArray,0);

    std::string playbackprefix = vm["playbackprefix"].as<std::string>();

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

        if (playbackprefix != "") {
        	graspSim->setStoreTimedStatePaths(true);
        }

        graspSim->load(grasptask);
        graspSim->startSimulation(initState);
        TimerUtil::sleepMs(2000);
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

        if (playbackprefix != "") {
        	std::map<GraspSubTask*,std::map<GraspTarget*,TimedStatePath> > timedStatePaths = graspSim->getTimedStatePaths();
        	BOOST_FOREACH(GraspSubTask &task, grasptask->getSubTasks()) {
        		if (timedStatePaths.find(&task) != timedStatePaths.end()) {
        			std::map<GraspTarget*,TimedStatePath> &targetPath = timedStatePaths[&task];
        			std::size_t i = 0;
        			BOOST_FOREACH(GraspTarget &target, task.getTargets()) {
                		if (targetPath.find(&target) != targetPath.end()) {
                			TimedStatePath &path = targetPath[&target];
                			std::stringstream sstr;
                			sstr << playbackprefix << ifile << task.getTaskID() << "_" << i << ".rwplay";
                			PathLoader::storeTimedStatePath(*dwc->getWorkcell(),path,sstr.str());
                		}
                		i++;
        			}
        		}
        	}
        }

    }
    std::cout << "Done" << std::endl;
    return 0;
}

#include <rwsim/util/SurfacePoseSampler.hpp>

GraspTask::Ptr generateTasks(int nrTasks, DynamicWorkCell::Ptr dwc, string objectName, string type){
    string gripperName;
    dwc->getBodies();
    WorkCell::Ptr _wc = dwc->getWorkcell();
    //Ununsed: int nrOfCollisions = 0;
    GraspTask::Ptr gtask = ownedPtr(new GraspTask());
    Body::Ptr body = dwc->findBody(objectName);
    if(body==NULL){
        RW_THROW("OBJECT DOES NOT EXIST: " << objectName);
    }
    std::vector<Geometry::Ptr> geoms = body->getGeometry();
    SurfacePoseSampler ssurf( geoms );
    ssurf.setRandomRotationEnabled(false);

    // these should be the object transformation
    Vector3D<> pos(0, 0, 0);
    Rotation3D<> rot(1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);


    // first set up the configuration
    Vector3D<> d(0,0,-0.02);
    Transform3D<> wTe_n(pos, rot);
    Transform3D<> wTe_home(pos+inverse(rot)*d, rot);
    Q openQ(1,0.0);
    Q closeQ(1,1.0);
    if( type=="PG70" ){
        openQ  = Q(1, 0.034);
        gripperName = type;
        closeQ = Q(1, 0.0);
        gtask->setTCPID("PG70.TCPPG70");
    } else if( type== "PG70_SMALL"){
        openQ  = Q(1, 0.01);
        closeQ = Q(1, 0.0);
        gripperName = type;
        gtask->setTCPID("PG70.TCPPG70");
    } else if( type== "GS20"){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
        gripperName = type;
        gtask->setTCPID("TCPGS20");
    } else if( type== "GS20_WIDE"){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
        gripperName = "GS20";
        gtask->setTCPID("TCPGS20");
    } else if( type== "SDH_PAR"){
        openQ =  Q(7,-1.571,-1.571,1.571, -1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        gtask->setTCPID("SchunkHand.SDHTCP");
        gripperName = "SchunkHand";
    } else if( type== "SDH_PAR1"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        gtask->setTCPID("SchunkHand.SDHTCP1");
        gripperName = "SchunkHand";
    } else if( type== "SDH_PAR2"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.1, 0.1, -0.1, 0.1);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        gtask->setTCPID("SchunkHand.SDHTCP1");
        gripperName = "SchunkHand";

    } else if( type== "SDH_PAR1_TABLE"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        gtask->setTCPID("SchunkHand.SDHTCP1");
        gripperName = "SchunkHand";
    } else if( type== "SDH_PAR2_TABLE"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.1, 0.1, -0.1, 0.1);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        gtask->setTCPID("SchunkHand.SDHTCP1");
        gripperName = "SchunkHand";

    } else if( type== "SDH_BALL"){
        openQ = Q(7,-1.048, 0.174, 1.047 ,-1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 1.047,0.0, 0.349,0.0, 0.349);
        gtask->setTCPID("SchunkHand.SDHTCP");
        gripperName = "SchunkHand";
    } else if( type== "SDH_CYL"){
        openQ = Q(7, -1.048, 0.174, 0.0, -1.048, 0.174,-1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 0.0, 0.0, 0.349, 0.0, 0.349);
        gtask->setTCPID("SchunkHand.SDHTCP");
        gripperName = "SchunkHand";
    } else if( type== "SCUP"){
        openQ  = Q(1, 0.0);
        closeQ = Q(1, 1.0);
        gtask->setTCPID("EndFrame");
        //_graspSim->setAlwaysResting(true);
        gripperName = type;
    } else {
        RW_THROW(" The gripper type is wrong! please specify a valid grippertype: (PG70, SCUP, SDH_PAR, SDH_CYL, SDH_BALL)");
    }



    //wTe_n = Transform3D<>::identity();
    //wTe_home = Transform3D<>::identity();
    gtask->setGripperID(gripperName);
    gtask->setGraspControllerID("GraspController");
    //rtask->getPropertyMap().set<std::string >("Object", objectName);

    //CartesianTask::Ptr tasks = ownedPtr( new CartesianTask());
    gtask->getSubTasks().resize(1);
    GraspSubTask &subtask = gtask->getSubTasks()[0];


    if( gripperName=="SchunkHand"){
        Q tau = Q(7, 2.0, 2.0, 10.0, 2.0, 2.0, 2.0, 2.0);
        // depending on the value of joint 2 adjust the forces
        double alpha = openQ(2);
        if(alpha<45*Deg2Rad){
            tau(3) = tau(0)/(2*cos(alpha));
            tau(5) = tau(0)/(2*cos(alpha));
        } else {
            tau(0) = std::max( 2*cos(alpha)*tau(3), 0.2);
        }
        subtask.tauMax = tau;
    }

    //rtask->addTask(tasks);

    subtask.offset = wTe_n;
    if( type== "SCUP"){
        subtask.approach = Transform3D<>(Vector3D<>(0,0,0.04));
        subtask.retract = Transform3D<>(Vector3D<>(0,0,0.0));
    } else if( gripperName=="GS20"){
        subtask.approach = Transform3D<>(Vector3D<>(0,0,0.0));
        subtask.retract = Transform3D<>(Vector3D<>(0,0,0.04));
    } else {
        subtask.approach = Transform3D<>(Vector3D<>(0,0,0.0));
        subtask.retract = Transform3D<>(Vector3D<>(0,0,0.10));
    }

    subtask.openQ = openQ;
    subtask.closeQ = closeQ;

    if( type=="GS20" || type=="GS20_WIDE"){
        ssurf.setBoundsD(-0.02,0.02);
    } else if( type=="SCUP" ){
        ssurf.setBoundsD(-0.02,0.005);
    } else {
        ssurf.setBoundsD(-0.04,0.04);
    }

    // now we choose a random number in the total area
    State state = dwc->getWorkcell()->getDefaultState();
    Transform3D<> wTo = rw::kinematics::Kinematics::worldTframe(body->getBodyFrame(), state);

    for(int i=0; i<nrTasks; i++){
        Transform3D<> target;

        target = wTo*ssurf.sample();
        subtask.targets.push_back( GraspTarget( target ) );
    }

    return gtask;
}


