

/**
 * This script is for modifying the VisGrab task files such that
 * 1. collision models for the point cloud data is generated
 * 2. the collision models are used to filter away initially colliding tasks
 * 3. change tasks to have correct CloseQ and correct TauMax
 *
 */


#include <sstream>
#include <vector>
#include <stack>
#include <string>
#include <stdio.h>
//#include <csignal>
#include <sys/stat.h>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>
#include <rwlibs/task/loader/XMLTaskSaver.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <boost/foreach.hpp>

using namespace std;
using namespace rw::common;
using rw::geometry::Geometry;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rwlibs::task;
using rwlibs::proximitystrategies::ProximityStrategyFactory;

std::vector<std::string> generateFileList(std::string,std::string,std::string, bool checkExistance);
std::vector<std::string> mergeTaskFileList(std::string root, std::string preName, std::string postName, bool forceMerge);
std::vector<std::string> merge(std::vector<std::string>, std::vector<std::string>);
void generateCollisionData();
void initVars();
std::vector<CartesianTask::Ptr> getAllTasks(CartesianTask::Ptr task);


string expRoot;
string toStlModel;
vector<string> graspTypes;
vector<string> objectDirectories;

int main(int argc, char** argv)
{
	//Unused: int directoryType = 0, sceneTypeStart = 1, mergeFiles = 0;
    expRoot = "";
    toStlModel = "imageFeaturesToStl";
    if (argc > 1) expRoot = std::string(argv[1]);
    if (argc > 2) toStlModel = std::string(argv[2]);

    initVars();

    std::cout << "\n\n\n";
    std::cout << "*********  Extend VisGrab Task Program ****************" << std::endl;
    std::cout << "\n\n\n";

    string iniFile = expRoot + "/RobWorkStudio.ini";
    string propFile = expRoot + "/singleObject.prop.xml";
    string dwcRoot = expRoot + "/workcells";
    string taskRoot = expRoot + "/tasks";
    string resultRoot = expRoot + "/results";

    std::cout << " ***************************************************** " << std::endl;
    bool FORCE_GENERATION = false;
    std::vector<std::string> non_textured_collision_files =
            generateFileList(expRoot+"/non-textured/collision", "img_", "_collisionInfo.dat", FORCE_GENERATION);
    std::vector<std::string> textured_collision_files =
            generateFileList(expRoot+"/textured/collision", "img_", "_collisionInfo.dat", FORCE_GENERATION);
    std::vector<std::string> colFiles = merge(non_textured_collision_files, textured_collision_files);
    std::vector<std::string> colStlFiles;
    std::cout << "* Generating Non textured Collision files: " << colFiles.size() << std::endl;
    bool FORCE_COLLISION_MODELS = false;
    BOOST_FOREACH(std::string file, colFiles){
        // test if its allready been generated
        char cmd2[1024];
        std::string outfile = file + ".stl";
        colStlFiles.push_back(outfile);
        std::cout <<"--> " << outfile << "\n";
        if(!FORCE_COLLISION_MODELS){
            struct stat stInfo;
            if (stat(outfile.c_str(), &stInfo) == 0)
                continue;
        }
        sprintf(cmd2, "%s %s 0.005 %s", toStlModel.c_str(), file.c_str(), outfile.c_str());
        //Unused: int ret = system(cmd2);

    }
    std::cout << "\n***************************************************** " << std::endl;

    // Next we need to merge the task files
    std::cout << " ***************************************************** " << std::endl;
    std::cout << " * Merging files " << std::endl;
    bool FORCE_MERGE = false;
    std::vector<std::string> taskFiles1 = mergeTaskFileList(expRoot+"/non-textured/tasks", "img_", ".task.xml", FORCE_MERGE);
    std::vector<std::string> taskFiles2 = mergeTaskFileList(expRoot+"/textured/tasks", "img_", ".task.xml", FORCE_MERGE);
    std::vector<std::string> taskFiles = merge(taskFiles1, taskFiles2);
    std::cout << " ***************************************************** " << std::endl;

    std::cout << " ***************************************************** " << std::endl;
    std::cout << " * Filtering tasks using collision detection" << std::endl;

    RW_ASSERT(taskFiles.size()==colStlFiles.size());
    // load workcell and initialize collision detector
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(expRoot+"/textured/workcells/empty.wc.xml");
    CollisionStrategy::Ptr strategy = ProximityStrategyFactory::makeDefaultCollisionStrategy();
    MovableFrame *_mframe = wc->findFrame<MovableFrame>("SchunkHand.Base");
    Frame *_end = wc->findFrame<Frame>("SchunkHand.Base");
    RW_ASSERT(_mframe);
    RW_ASSERT(_end);
    State state = wc->getDefaultState();
    rw::models::Device::Ptr hand = wc->findDevice<Device>("SchunkHand");
    Transform3D<> _bTe = Kinematics::frameTframe(_mframe, _end, state);
    size_t nrOfCollisions=0, nrOfTargets=0, nrOutOfBounds = 0;
    for(size_t i=0;i<colStlFiles.size();i++){
        std::string taskFile = taskFiles[i];
        std::string colFile = colStlFiles[i];
        size_t nrCollisionsTask = 0, nrTargetsTask = 0, nrOutOfBoundsTask=0;

        // load the collision model for a specific merged task
        CollisionDetector::Ptr coldect = ownedPtr(new CollisionDetector(wc, strategy));

        Geometry::Ptr geom = GeometryFactory::load(colFile, false);
        geom->setId("objectGeom");
        coldect->addGeometry(wc->getWorldFrame(), geom);


        // load the task
        std::cout << "* Task: " << taskFile.substr(taskFile.size()-40,40) << std::endl;
        std::cout << "* Task: " << taskFile<< std::endl;
        // for each task set the hand in its initial configuration and remove all targets that are in collision

        XMLTaskLoader loader;
        loader.load(taskFile);

        CartesianTask::Ptr filterTask = ownedPtr(new CartesianTask());
        std::cout << "" << std::endl;
        CartesianTask::Ptr rootTask = loader.getCartesianTask();
        std::vector<CartesianTask::Ptr> allTasks = getAllTasks( rootTask );
        std::vector<CartesianTask::Ptr> filteredTasks;

        BOOST_FOREACH(CartesianTask::Ptr task, allTasks){
            Transform3D<> _wTe_n = task->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
            //_wTe_home = _currenttask->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
            //Vector3D<> approach = _currenttask->getPropertyMap().get<Vector3D<> >("Approach", Vector3D<>(0,0,0));
            //_approachDef = Transform3D<>( approach, Rotation3D<>::identity());
            Q _openQ = task->getPropertyMap().get<Q>("OpenQ", Q(7,0.0));
            Q _closeQ = task->getPropertyMap().get<Q>("CloseQ", Q(7,0.0));
            if( task->getPropertyMap().get<int>("GraspTypeI",2)<2 ){
                // we modify the closed preshape
                _closeQ = Q(7,-1.571,-1.571,1.571,0.0,0.419,0.0,0.419);
                task->getPropertyMap().set<Q>("CloseQ", _closeQ);
            }

            if( _openQ(2)<-0.001 || _openQ(2)> Pi/2+1*Deg2Rad ){
                //std::cout << _openQ(2) << std::endl;
                nrOutOfBoundsTask++;
                nrOutOfBounds++;
                continue;
            }

            Q low(7, -Pi/2, -Pi/2, 0.0, -Pi/2, -Pi/2, -Pi/2, -Pi/2);
            Q upper(7, Pi/2, Pi/2, Pi/2, Pi/2, Pi/2, Pi/2, Pi/2);
            _openQ = Math::clampQ(_openQ, low, upper);
            task->getPropertyMap().set<Q>("OpenQ", _openQ);

            Q handForceLimits = Q(7, 2.1, 1.4, 2.1, 2.1, 1.4, 2.1, 1.4);
            double alpha = _openQ(2);
            if(alpha<=Pi/4){
                // if the two fingers are pointing toward (45 degree) the thumb then reduce
                // the fingers forces according to the max force of the thumb
                handForceLimits(3) = std::max( handForceLimits(0)/(2*cos(alpha)), 0.1);
                handForceLimits(4) = std::max( 0.1, handForceLimits(1)/(2*cos(alpha)) );
                handForceLimits(5) = std::max( 0.1,handForceLimits(0)/(2*cos(alpha)) );
                handForceLimits(6) = std::max( 0.1,handForceLimits(1)/(2*cos(alpha)) );
            } else {
                // else reduce the thumb force such that it does not push the object away
                handForceLimits(0) = std::max( 0.1, 2*cos(alpha)*handForceLimits(3));
                handForceLimits(1) = std::max( 0.1, 2*cos(alpha)*handForceLimits(4));
            }
            task->getPropertyMap().set<Q>("TauMax", handForceLimits);


            std::vector<CartesianTarget::Ptr> filteredTargets;
            Transform3D<> wTp = Kinematics::worldTframe(_mframe->getParent(state), state);
            BOOST_FOREACH(CartesianTarget::Ptr target, task->getTargets() ){
                Transform3D<> start = inverse(wTp) * _wTe_n * target->get() * inverse(_bTe);
                nrOfTargets++;
                nrTargetsTask++;
                _mframe->setTransform(start, state);
                hand->setQ(_openQ, state);
                if( !coldect->inCollision(state, NULL, true) ){
                    filteredTargets.push_back(target);
                } else {
                    nrOfCollisions++;
                    nrCollisionsTask++;
                }
            }
            task->getTasks().clear();
            task->getTargets() = filteredTargets;
            if(task->getTargets().size()>0 && task!=rootTask){
                //std::cout << "targets: " << filteredTargets.size() << "==" << task->getTargets().size()<< "\n";
                //filteredTasks.push_back(task);
                filterTask->addTask(task);
            }
        }        
        
        rootTask->getTasks() = filteredTasks;
        try {
            XMLTaskSaver saver;
            std::stringstream sstr;
            sstr << taskFile << ".filtered.xml";
            saver.save(filterTask, sstr.str() );
            //RW_WARN("");
        } catch (const Exception& exp) {

            // QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
        }

        std::cout << "* Targets:" << nrTargetsTask << ", collisions:" << nrCollisionsTask << ", outBounds:" << nrOutOfBoundsTask << ", Left:" << (1.0*nrTargetsTask-nrCollisionsTask)/nrTargetsTask*100.0<< "%" << std::endl;

    }
    std::cout << "* Total nr of targets: " << nrOfTargets << std::endl;
    std::cout << "* Nr of targets that where in collision: " << nrOfCollisions << std::endl;
    std::cout << "* Nr of targets that where out of bounds: " << nrOutOfBounds << std::endl;
    std::cout << "* Percentage of good targets: " << (nrOfTargets-(nrOutOfBounds+nrOfCollisions)*1.0)/nrOfTargets*100.0 << "%" << std::endl;

    std::cout << "***************************************************** " << std::endl;


    return (0);
}

void generateCollisionData(){

}

std::vector<CartesianTask::Ptr> getAllTasks(CartesianTask::Ptr task){
    int nrOfTargets = 0;
    std::vector<CartesianTask::Ptr> alltasks;
    std::stack<rwlibs::task::CartesianTask::Ptr> tmpStack;
    tmpStack.push(task);
    while(!tmpStack.empty()){
        rwlibs::task::CartesianTask::Ptr tmpTask = tmpStack.top();
        tmpStack.pop();
        alltasks.push_back(tmpTask);
        nrOfTargets += tmpTask->getTargets().size();
        BOOST_FOREACH(rwlibs::task::CartesianTask::Ptr subtask, tmpTask->getTasks()){
            tmpStack.push(subtask);
        }
    }
    return alltasks;
}

void initVars(){
    graspTypes.push_back("twofingeredgegrasp");
    graspTypes.push_back("twofingersurfacegrasp");
    graspTypes.push_back("threefingersurfacegrasp");

    objectDirectories.push_back("amicelli");
    objectDirectories.push_back("amicelli_whitecup");
    objectDirectories.push_back("breadbox");
    objectDirectories.push_back("breadbox_peachcan");
    objectDirectories.push_back("burti");
    objectDirectories.push_back("burti_denkmit");
    objectDirectories.push_back("cleaningbottle");
    objectDirectories.push_back("cleaningbottle_sucker");
    objectDirectories.push_back("cocacola");
    objectDirectories.push_back("cocacola_saltcylinder");
    objectDirectories.push_back("corny");
    objectDirectories.push_back("corny_greencup");
    objectDirectories.push_back("denkmit");
    objectDirectories.push_back("greencup");
    objectDirectories.push_back("icecoffee");
    objectDirectories.push_back("icecoffee_saltbox");
    objectDirectories.push_back("marmelade");
    objectDirectories.push_back("marmelade_pure");
    objectDirectories.push_back("peachcan");
    objectDirectories.push_back("pure");
    objectDirectories.push_back("saltbox");
    objectDirectories.push_back("saltcylinder");
    objectDirectories.push_back("sucker");
    objectDirectories.push_back("tomato");
    objectDirectories.push_back("tomato_waterjug");
    objectDirectories.push_back("waterjug");
    objectDirectories.push_back("whitecup");
}

std::vector<std::string> generateFileList(std::string root, std::string preName, std::string postName, bool foreGen){
    int nrOfScenesPerObject = 8;
    std::vector<std::string> files;
    // first we generate all collision data
    for (unsigned int objI = 0; objI < objectDirectories.size(); objI++) {
        string objectDirectory = objectDirectories[objI];
        // each of these can be merged to one file...
        // we call it
        for (int sceneI = 1; sceneI <= nrOfScenesPerObject; sceneI++) {
            char sIstr[10];
            sprintf(sIstr, "%02d", sceneI);
            string file = root + "/" + objectDirectory + "/" + preName + sIstr + postName;
            if(!foreGen){
                struct stat stInfo;
                if (stat(file.c_str(), &stInfo) != 0)
                    continue;
            }

            files.push_back(file);
        }
    }
    return files;
}


std::vector<std::string> mergeTaskFileList(std::string root, std::string preName, std::string postName, bool forceMerge){
    int nrExp = 0;

    int nrOfScenesPerObject = 8;
    std::vector<std::string> files;
    // first we generate all collision data
    for (unsigned int objI = 0; objI < objectDirectories.size(); objI++) {
        string objectDirectory = objectDirectories[objI];
        // each of these can be merged to one file...
        // we call it

        for (int sceneI = 1; sceneI <= nrOfScenesPerObject; sceneI++) {
            char sIstr[10];
            sprintf(sIstr, "%02d", sceneI);

            //string dwcFile = root + "/" + objectDirectory + "/" + "img_" + sIstr + ".dwc.xml";
            string merged_taskFile = root + "/" + objectDirectory + "/" + "merged_" + sIstr + ".task.xml";
            files.push_back(merged_taskFile);

            // test if the file allready exist
            struct stat stInfo;
            if (stat(merged_taskFile.c_str(), &stInfo) == 0 && !forceMerge){
                std::cout << ".";
                continue;
            }

            rwlibs::task::CartesianTask::Ptr tasks = ownedPtr( new rwlibs::task::CartesianTask() );
            int nrExpMerged = 0;
            //std::cout << "-- Compiling tasks into: " << std::string("/" + objectDirectory + "/" + "merged_" + sIstr + ".task.xml") << std::endl;

            for(size_t gtype=0; gtype<graspTypes.size(); gtype++){
                string graspType = graspTypes[gtype];
                int graspI = 0;
                while (true) {
                    char gIstr[10];
                    sprintf(gIstr, "%03d", graspI);

                    string file = root + "/" + objectDirectory + "/" + preName + sIstr + "_" + graspType + "_" + gIstr + postName;

                    struct stat stInfo;
                    if (stat(file.c_str(), &stInfo) != 0)
                        break;

                    // add it to the task
                    try {
                        XMLTaskLoader loader;
                        loader.load( file );
                        rwlibs::task::CartesianTask::Ptr task = loader.getCartesianTask();
                        task->setId( objectDirectory + "_" + "img_" + sIstr + "_" + graspType + "_" + gIstr );
                        task->getPropertyMap().set<string>("GraspType",graspType);
                        task->getPropertyMap().set<int>("GraspTypeI",gtype);
                        task->getPropertyMap().set<string>("ObjectType",objectDirectory);
                        task->getPropertyMap().set<int>("GraspI",graspI);
                        task->getPropertyMap().set<string>("DWCFile", string(objectDirectory + "/" + "img_" + sIstr + ".dwc.xml"));
                        task->getPropertyMap().set<int>("SceneI",sceneI);
                        tasks->addTask( task );
                    } catch (const Exception& exp) {
                        RW_WARN("task file not loaded!");
                    }
                    nrExp++;
                    nrExpMerged++;

                    graspI++;
                }
                cout << "-- " << graspType << ", " << objectDirectory << ", sceneI: " << sceneI << ", nrGrasps: " << graspI << endl;
            }

            std::cout << "Merging " << nrExpMerged << " tasks into: " << std::string("/" + objectDirectory + "/" + "merged_" + sIstr + ".task.xml") << std::endl;
            try {
                XMLTaskSaver saver;
                saver.save(tasks, merged_taskFile );
            } catch (const Exception& exp) {
               // QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
            }
        }
    }
    std::cout << "Total task nr merged: " << nrExp << std::endl;
    return files;
}



std::vector<std::string> merge(std::vector<std::string> v1, std::vector<std::string> v2){
    std::vector<std::string> result = v1;
    BOOST_FOREACH(std::string str, v2){
        result.push_back(str);
    }
    return result;
}


#ifdef OLDBYM

    int nrExp = 0;
    bool mergeTaskFiles = mergeFiles==1;
    if(mergeTaskFiles){
        for (unsigned int objI = directoryType; objI < objectDirectories.size(); objI++) {
            string objectDirectory = objectDirectories[objI];

            // each of these can be merged to one file...
            // we call it
            for (int sceneI = sceneTypeStart; sceneI <= nrOfScenesPerObject; sceneI++) {
                char sIstr[10];
                sprintf(sIstr, "%02d", sceneI);
                string dwcFile = dwcRoot + "/" + objectDirectory + "/" + "img_" + sIstr + ".dwc.xml";

                string merged_taskFile = taskRoot + "/" + objectDirectory + "/" + "merged_" + sIstr + ".task.xml";
                rwlibs::task::CartesianTask::Ptr tasks = ownedPtr( new rwlibs::task::CartesianTask() );
                int nrExpMerged = 0;
                std::cout << "Compiling tasks into: " << merged_taskFile << std::endl;
                for (unsigned int graspTypeI = 0; graspTypeI < graspTypes.size(); graspTypeI++) {
                    string graspType = graspTypes[graspTypeI];


                    int graspI = 0;
                    while (true) {
                        char gIstr[10];
                        sprintf(gIstr, "%03d", graspI);
                        string taskFile = taskRoot + "/" + objectDirectory + "/" + "img_" + sIstr + "_" + graspType + "_"
                                + gIstr + ".task.xml";

                        struct stat stInfo;
                        if (stat(taskFile.c_str(), &stInfo) != 0) break;

                        string resultFile = resultRoot + "/" + objectDirectory + "/" + "img_" + sIstr + "_" + graspType
                                + "_" + gIstr + ".res.xml";

                        try {
                            XMLTaskLoader loader;
                            loader.load( taskFile );
                            rwlibs::task::CartesianTask::Ptr task = loader.getCartesianTask();
                            task->setId( objectDirectory + "_" + "img_" + sIstr + "_" + graspType + "_" + gIstr );
                            task->getPropertyMap().set<string>("GraspType",graspType);
                            task->getPropertyMap().set<int>("GraspTypeI",graspTypeI);
                            task->getPropertyMap().set<string>("ObjectType",objectDirectory);
                            task->getPropertyMap().set<int>("GraspI",graspI);
                            task->getPropertyMap().set<string>("DWCFile", string(objectDirectory + "/" + "img_" + sIstr + ".dwc.xml"));
                            task->getPropertyMap().set<int>("SceneI",sceneI);
                            tasks->addTask( task );
                        } catch (const Exception& exp) {
                            RW_WARN("task file not loaded!");
                        }
                        nrExp++;
                        nrExpMerged++;

                        graspI++;
                    }

                    /*
                    XMLTaskLoader loader;
                    loader.load( taskFile );
                    rwlibs::task::CartesianTask::Ptr task = loader.getCartesianTask();
                    std::vector<rwlibs::task::CartesianTask::Ptr> subtasks = task->getTasks();
                    BOOST_FOREACH(rwlibs::task::CartesianTask::Ptr subtask, subtasks){
                        std::cout << subtask->getId() << std::endl;
                        std::cout << subtask->getPropertyMap()->get<string>("GraspType","") << std::endl;
                        std::cout << subtask->getPropertyMap()->get<int>("GraspTypeI",-1) << std::endl;
                        std::cout << subtask->getPropertyMap()->get<string>("ObjectType","") << std::endl;
                        std::cout << subtask->getPropertyMap()->get<int>("GraspI",-1) << std::endl;
                        std::cout << subtask->getPropertyMap()->get<string>("DWCFile","") << std::endl;
                        std::cout << subtask->getPropertyMap()->get<int>("SceneI",-1) << std::endl;
                    }
                    */

                    //if (objI == 1) exit(1);
                    cout << "-- " << graspType << ", " << objectDirectory << ", sceneI: " << sceneI << ", nrGrasps: " << graspI << endl;
                }

                std::cout << "Merging " << nrExpMerged << " tasks into: " << merged_taskFile << std::endl;
                try {
                    XMLTaskSaver saver;
                    saver.save(tasks, merged_taskFile );
                } catch (const Exception& exp) {
                   // QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
                }

            }

        }

        std::cout << "Total task nr: " << nrExp << std::endl;
        return 0;
    }

    bool use_existing_result_file = mergeFiles==2;

    unsigned int endIdx = directoryType+1; objectDirectories.size();
    unsigned int startIdxDType = directoryType;
    if(directoryType<0){
        endIdx = objectDirectories.size();
        startIdxDType = 0;
    }

    for (unsigned int objI = startIdxDType ; objI < endIdx; objI++) {
        string objectDirectory = objectDirectories[objI];

        // each of these can be merged to one file...
        // we call it
        for (int sceneI = sceneTypeStart; sceneI <= nrOfScenesPerObject; sceneI++) {

            char sIstr[10];
            sprintf(sIstr, "%02d", sceneI);
            string dwcFile = dwcRoot + "/" + objectDirectory + "/" + "img_" + sIstr + ".dwc.xml";

            string merged_taskFile = taskRoot + "/" + objectDirectory + "/" + "merged_" + sIstr + ".task.xml";

/*
            char gIstr[10];
            sprintf(gIstr, "%03d", graspI);
            string taskFile = taskRoot + "/" + objectDirectory + "/" + "img_" + sIstr + "_" + graspType + "_"
                    + gIstr + ".task.xml";
*/
            struct stat stInfo;
            if (stat(merged_taskFile.c_str(), &stInfo) != 0) break;

            string resultFile = resultRoot + "/" + objectDirectory + "/" + "merged_" + sIstr + ".res.xml";

            if(use_existing_result_file){
                if (stat(resultFile.c_str(), &stInfo) == 0){
                    cout << "CONTINUEING A RESULT FILE - USING RESULTFILE AS TASK FILE" << std::endl;
                    merged_taskFile = resultFile;
                }
            } else {
                // if result file exist allready then skip it
                if (stat(resultFile.c_str(), &stInfo) == 0){
                    cout << "*************************************************************" << std::endl;
                    cout << "* SKIPPED: [" << objI <<"," << sceneI <<"] " << objectDirectory << "_img_" << string(sIstr) << std::endl;
                    cout << "*************************************************************" << std::endl;
                    continue;
                }
            }
            // Execute RobWork
            char cmd2[1024];
            cout << "*************************************************************" << std::endl;
            cout << "* STARTED: [" << objI <<"," << sceneI <<"] " << objectDirectory << "_img_" << string(sIstr) << std::endl;
            cout << "* " << std::endl;
            sprintf(
                    cmd2,
                    "start \"%s\" /WAIT %s --ini-file=%s -PDWC=%s -PSimTaskConfig=%s -PTaskTestFile=%s -PTaskTestOutFile=%s -PAuto=True -PNoSave=True",
                    merged_taskFile.c_str(),
                    robWorkStudio.c_str(), iniFile.c_str(), dwcFile.c_str(), propFile.c_str(),
                    merged_taskFile.c_str(), resultFile.c_str());
            cout << cmd2 << endl;
            int ret = system(cmd2);
            cout << "*************************************************************" << std::endl;
            //if (WIFSIGNALED(ret) && (WTERMSIG(ret) == SIGINT || WTERMSIG(ret) == SIGQUIT))
            //    exit(EXIT_FAILURE);

            nrExp++;

        }
    }

    /*

    for (unsigned int graspTypeI = startIdxGType; graspTypeI < graspTypes.size(); graspTypeI++) {
        string graspType = graspTypes[graspTypeI];

        for (unsigned int objI = startIdxDType; objI < objectDirectories.size(); objI++) {
            string objectDirectory = objectDirectories[objI];

            for (int sceneI = 1; sceneI <= nrOfScenesPerObject; sceneI++) {
                char sIstr[10];
                sprintf(sIstr, "%02d", sceneI);
                string dwcFile = dwcRoot + "/" + objectDirectory + "/" + "img_" + sIstr + ".dwc.xml";

                int graspI = 0;
                while (true) {
                    char gIstr[10];
                    sprintf(gIstr, "%03d", graspI);
                    string taskFile = taskRoot + "/" + objectDirectory + "/" + "img_" + sIstr + "_" + graspType + "_"
                            + gIstr + ".task.xml";

                    struct stat stInfo;
                    if (stat(taskFile.c_str(), &stInfo) != 0) break;

                    string resultFile = resultRoot + "/" + objectDirectory + "/" + "img_" + sIstr + "_" + graspType
                            + "_" + gIstr + ".res.xml";

                    // Execute RobWork
                    char cmd2[1024];

                    sprintf(
                            cmd2,
                            "%s --ini-file=%s -PDWC=%s -PSimTaskConfig=%s -PTaskTestFile=%s -PTaskTestOutFile=%s -PAuto=True -PNoSave=True",
                            robWorkStudio.c_str(), iniFile.c_str(), dwcFile.c_str(), propFile.c_str(),
                            taskFile.c_str(), resultFile.c_str());
                    cout << cmd2 << endl;
                    int ret = system(cmd2);
                    //if (WIFSIGNALED(ret) && (WTERMSIG(ret) == SIGINT || WTERMSIG(ret) == SIGQUIT))
                    //    exit(EXIT_FAILURE);

                    nrExp++;

                    graspI++;
                }
                //if (objI == 1) exit(1);
                cout << graspType << ", " << objectDirectory << ", sceneI: " << sceneI << ", nrGrasps: " << graspI
                        << endl;

            }
        }
    }
    */

    cout << "Nr of experiments: " << nrExp << endl;
#endif

