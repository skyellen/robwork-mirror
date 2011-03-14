
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

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;


int main(int argc, char** argv)
{
    int directoryType = 0, sceneTypeStart = 1, mergeFiles = 0;
    string expRoot = "";
    string robWorkStudio = "RobWorkStudio";
    if (argc > 1) expRoot = std::string(argv[1]);
    if (argc > 2) robWorkStudio = std::string(argv[2]);
    if (argc > 3) directoryType = (int)std::atoi(argv[3]);
    if (argc > 4) sceneTypeStart = (int)std::atoi(argv[4]);
    if (argc > 5) mergeFiles = (int)std::atoi(argv[5]);
    if(sceneTypeStart<1)
        sceneTypeStart = 1;

    std::cout << "\n\n\n";
    std::cout << "*********  Simulate Multi Task Program ****************" << std::endl;
    std::cout << "\n\n\n";

    string iniFile = expRoot + "/RobWorkStudio.ini";
    string propFile = expRoot + "/singleObject.prop.xml";
    string dwcRoot = expRoot + "/workcells";
    string taskRoot = expRoot + "/tasks";
    string resultRoot = expRoot + "/results";

    vector<string> graspTypes;
    graspTypes.push_back("twofingeredgegrasp");
    graspTypes.push_back("twofingersurfacegrasp");
    graspTypes.push_back("threefingersurfacegrasp");

    vector<string> objectDirectories;
    objectDirectories.push_back("amicelli");
    objectDirectories.push_back("breadbox");
    objectDirectories.push_back("burti");
    objectDirectories.push_back("denkmit");
    objectDirectories.push_back("greencup");
    objectDirectories.push_back("mangocan");
    objectDirectories.push_back("peachcan");
    objectDirectories.push_back("saltbox");
    objectDirectories.push_back("saltcylinder");
    objectDirectories.push_back("tomato");
    objectDirectories.push_back("whitecup");

    int nrOfScenesPerObject = 16;
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

    return (0);
}
