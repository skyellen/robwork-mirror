

/**
 * This script is for modifying the VisGrab task files such that
 * 1. collision models for the point cloud data is generated
 * 2. the collision models are used to filter away initially colliding tasks
 * 3. change tasks to have correct CloseQ and correct TauMax
 *
 */


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

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>


USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rwlibs::proximitystrategies;

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
    int directoryType = 0, sceneTypeStart = 1, mergeFiles = 0;
    expRoot = "";
    toStlModel = "imageFeaturesToStl";
    if (argc > 1) expRoot = std::string(argv[1]);
    if (argc > 2) toStlModel = std::string(argv[2]);

    initVars();

    //if (argc > 3) directoryType = (int)std::atoi(argv[3]);
    //if (argc > 4) sceneTypeStart = (int)std::atoi(argv[4]);
    //if (argc > 5) mergeFiles = (int)std::atoi(argv[5]);
    //if(sceneTypeStart<1)
    //    sceneTypeStart = 1;

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
    bool FORCE_COLLISION_MODELS = true;
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
        int ret = system(cmd2);

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
    WorkCell::Ptr wc = WorkCellLoader::load(expRoot+"/textured/workcells/empty.wc.xml");
    CollisionStrategy::Ptr strategy = ProximityStrategyFactory::makeDefaultCollisionStrategy();
    MovableFrame *_mframe = wc->findFrame<MovableFrame>("SchunkHand.Base");
    Frame *_end = wc->findFrame<Frame>("SchunkHand.TCP");
    RW_ASSERT(_mframe);
    RW_ASSERT(_end);
    State state = wc->getDefaultState();
    rw::models::Device::Ptr hand = wc->findDevice<Device>("SchunkHand");
    Transform3D<> _bTe = Kinematics::frameTframe(_mframe, _end, state);
    size_t nrOfCollisions=0, nrOfTargets=0;
    for(size_t i=0;i<colStlFiles.size();i++){
        std::string taskFile = taskFiles[i];
        std::string colFile = colStlFiles[i];
        size_t nrCollisionsTask = 0,nrTargetsTask = 0;

        // load the collision model for a specific merged task
        CollisionDetector::Ptr coldect = ownedPtr(new CollisionDetector(wc, strategy));

        Geometry::Ptr geom = GeometryFactory::load(colFile, false);
        geom->setId("objectGeom");
        coldect->addGeometry(wc->getWorldFrame(), geom);


        // load the task
        std::cout << "* Task: " << taskFile.substr(taskFile.size()-40,40) << std::endl;
        // for each task set the hand in its initial configuration and remove all targets that are in collision

        XMLTaskLoader loader;
        loader.load(taskFile);
        CartesianTask::Ptr rootTask = loader.getCartesianTask();
        std::vector<CartesianTask::Ptr> allTasks = getAllTasks( rootTask );
        BOOST_FOREACH(CartesianTask::Ptr task, allTasks){
            Transform3D<> _wTe_n = task->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
            //_wTe_home = _currenttask->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
            //Vector3D<> approach = _currenttask->getPropertyMap().get<Vector3D<> >("Approach", Vector3D<>(0,0,0));
            //_approachDef = Transform3D<>( approach, Rotation3D<>::identity());
            Q _openQ = task->getPropertyMap().get<Q>("OpenQ", Q(7,0.0));
            Q _closeQ = task->getPropertyMap().get<Q>("CloseQ", _closeQ);
            if( task->getPropertyMap().get<int>("GraspTypeI",2)<2 ){
                // we modify the closed preshape
                _closeQ = Q(7,-1.571,-1.571,1.571,0,0.419,0,0.419);
            }


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
            task->getTargets() = filteredTargets;
        }

        try {
            XMLTaskSaver saver;
            std::stringstream sstr;
            sstr << taskFile << ".filtered.xml";

            saver.save(rootTask, sstr.str() );
        } catch (const Exception& exp) {
           // QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
        }

        std::cout << "* Targets:" << nrTargetsTask << ", collisions:" << nrCollisionsTask << ", Left:" << (1.0*nrTargetsTask-nrCollisionsTask)/nrTargetsTask*100.0<< "%" << std::endl;

    }
    std::cout << "* Total nr of targets: " << nrOfTargets << std::endl;
    std::cout << "* Nr of targets that where in collision: " << nrOfCollisions << std::endl;
    std::cout << "* Percentage of good targets: " << (nrOfTargets-nrOfCollisions*1.0)/nrOfTargets*100.0 << "%" << std::endl;

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
        for (int sceneI = 0; sceneI <= nrOfScenesPerObject; sceneI++) {
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

