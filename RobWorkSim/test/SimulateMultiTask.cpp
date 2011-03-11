#include <windows.h>
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
    int startIdxGType = 0, startIdxDType = 0, mergeFiles = 0;
    if (argc > 1) startIdxGType = (int)std::atoi(argv[1]);
    if (argc > 2) startIdxDType = (int)std::atoi(argv[2]);
    if (argc > 3) mergeFiles = (int)std::atoi(argv[3]);

    string robWorkStudio = "c:/local/rwworkspace/RobWorkStudio/bin/Release/RobWorkStudio";

    string expRoot = "c:/Users/jimali/Downloads/IROS2011/IROS2011";
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
    bool mergeTaskFiles = mergeFiles!=0;
    if(mergeTaskFiles){
        for (unsigned int objI = startIdxDType; objI < objectDirectories.size(); objI++) {
            string objectDirectory = objectDirectories[objI];

            // each of these can be merged to one file...
            // we call it



            for (int sceneI = 1; sceneI <= nrOfScenesPerObject; sceneI++) {
                char sIstr[10];
                sprintf(sIstr, "%02d", sceneI);
                string dwcFile = dwcRoot + "/" + objectDirectory + "/" + "img_" + sIstr + ".dwc.xml";

                string merged_taskFile = taskRoot + "/" + objectDirectory + "/" + "merged_" + sIstr + ".task.xml";
                rwlibs::task::CartesianTask::Ptr tasks = ownedPtr( new rwlibs::task::CartesianTask() );
                int nrExpMerged = 0;
                std::cout << "Compiling tasks into: " << merged_taskFile << std::endl;
                for (unsigned int graspTypeI = startIdxGType; graspTypeI < graspTypes.size(); graspTypeI++) {
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
                            tasks->addTask( task );
                        } catch (const Exception& exp) {
                            RW_WARN("task file not loaded!");
                        }
                        nrExp++;
                        nrExpMerged++;

                        graspI++;
                    }
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

    cout << "Nr of experiments: " << nrExp << endl;

    return (0);
}
