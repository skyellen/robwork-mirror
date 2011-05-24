
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

#include <iostream>
#include <boost/filesystem.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace boost::filesystem;

std::vector<std::string> generateFileList(std::string,std::string,std::string, bool checkExistance);
std::vector<std::string> merge(std::vector<std::string>, std::vector<std::string>);
void initVars();
std::vector<CartesianTask::Ptr> getAllTasks(CartesianTask::Ptr task);
bool hasUndoneTasks(std::string, bool &makeNew);
int getNrOfTargets(std::string taskfile);


string expRoot;
string toStlModel;
vector<string> graspTypes;
vector<string> objectDirectories;

int main(int argc, char** argv)
{

    expRoot = "";
    string robWorkStudio = "RobWorkStudio";
    string robWorkStudioini = "RobWorkStudio.ini";
    if(argc<4){
        std::cout << "HELP " << std::endl;
        std::cout << "Arg 1: root to BenchMarkFolder" << std::endl;
        std::cout << "Arg 2: full path/name of RobWorkStudio executable" << std::endl;
        std::cout << "Arg 3: full path/name of RobWorkStudio inifile" << std::endl;
        return 0;
    }

    if (argc > 1) expRoot = std::string(argv[1]);
    if (argc > 2) robWorkStudio = std::string(argv[2]);
    if (argc > 3) robWorkStudioini = std::string(argv[3]);

    path rootpath(expRoot);
    if(!exists(rootpath)){
        std::cout << " folder does not exist! : " << expRoot << std::endl;
        return 0;
    }
    if(!is_directory(rootpath)){
        std::cout << " folder is not a directory!" << std::endl;
        return 0;
    }

    std::cout << rootpath.relative_path() << std::endl;
    initVars();

    std::cout << "\n\n\n";
    std::cout << "*********  Simulate Multi Task Program ****************" << std::endl;
    std::cout << "\n\n\n";

    string iniFile = robWorkStudioini; //iniexpRoot + "/RobWorkStudio.ini";
    string propFile = expRoot + "/textured/singleObject.prop.xml";
    //string dwcRoot = expRoot + "/workcells";
    //string taskRoot = expRoot + "/tasks";
    //string resultRoot = expRoot + "/results";

    bool FORCE_GENERATION = true;
    std::vector<std::string> non_textured_files =
            generateFileList(expRoot+"/non-textured/tasks", "merged_", ".task.xml.filtered.xml", FORCE_GENERATION);
    std::vector<std::string> textured_files =
            generateFileList(expRoot+"/textured/tasks", "merged_", ".task.xml.filtered.xml", FORCE_GENERATION);
    std::vector<std::string> taskFiles = merge(non_textured_files, textured_files);

    std::vector<std::string> non_textured_dwcfiles =
            generateFileList(expRoot+"/non-textured/workcells", "img_", ".dwc.xml", FORCE_GENERATION);
    std::vector<std::string> textured_dwcfiles =
            generateFileList(expRoot+"/textured/workcells", "img_", ".dwc.xml", FORCE_GENERATION);
    std::vector<std::string> dwcFiles = merge(non_textured_dwcfiles, textured_dwcfiles);

    std::vector<std::string> non_textured_resultfiles =
            generateFileList(expRoot+"/non-textured/results", "img_", "_result.task.xml", FORCE_GENERATION);
    std::vector<std::string> textured_resultfiles =
            generateFileList(expRoot+"/textured/results", "img_", "_result.task.xml", FORCE_GENERATION);
    std::vector<std::string> resultFiles = merge(non_textured_resultfiles, textured_resultfiles);
    std::cout << "DO SIM " << std::endl;
    int nrExp = 0;
    Timer time;
    bool FORCE_TASK_SIMULATION = false;
    for(size_t idx = 0; idx<taskFiles.size(); idx++){
        string taskFile = taskFiles[idx];
        string dwcFile = dwcFiles[idx];
        string resultFile = resultFiles[idx];
        string runFile = taskFile; // the file to run
        cout << "*************************************************************" << std::endl;
        cout << "* TASK: [" << taskFile << std::endl;
        bool makeNew = true;
        // if the result file exists then check if there are still tasks to be simulated
        struct stat stInfo;
        // if the file exists and we want to continue from where we left then we need to run the result file
        if(! FORCE_TASK_SIMULATION){
            if (stat(resultFile.c_str(), &stInfo) == 0){
                if(! hasUndoneTasks(resultFile, makeNew) ){
                    cout << "* SKIPPED: all results done!" << std::endl;
                    cout << "*************************************************************" << std::endl;
                    continue;
                }
                //if(!makeNew)
                    runFile = resultFile;
                cout << "* CONTINUEING A RESULT FILE - USING RESULTFILE AS TASK FILE" << std::endl;
            }
        }

        int nrOfTargets = getNrOfTargets(runFile);

        // Execute RobWork
        char cmd2[1024];
        cout << "* STARTING, nr of targets: " << nrOfTargets << std::endl;
        cout << "* " << std::endl;
        sprintf(
                cmd2,
                "start \"%s\" /WAIT %s --ini-file=%s -PDWC=%s -PSimTaskConfig=%s -PTaskTestFile=%s -PTaskTestOutFile=%s -PAuto=True -PNoSave=True",
                runFile.substr(runFile.size()-40,40).c_str(),
                robWorkStudio.c_str(),
                iniFile.c_str(),
                dwcFile.c_str(),
                propFile.c_str(),
                runFile.c_str(),
                resultFile.c_str());
        cout << cmd2 << endl;
        int ret = system(cmd2);
        cout << "*************************************************************" << std::endl;
        //if (WIFSIGNALED(ret) && (WTERMSIG(ret) == SIGINT || WTERMSIG(ret) == SIGQUIT))
        //    exit(EXIT_FAILURE);

        nrExp += nrOfTargets;
    }

    std::cout << "Time: "<<  time.toString() << std::endl;
    cout << "Nr of experiments: " << nrExp << endl;

    return (0);
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


std::vector<std::string> merge(std::vector<std::string> v1, std::vector<std::string> v2){
    std::vector<std::string> result = v1;
    BOOST_FOREACH(std::string str, v2){
        result.push_back(str);
    }
    return result;
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

bool hasUndoneTasks(std::string taskfile, bool& makeNewFile){
    struct stat stInfo;
    makeNewFile = true;
    // if the file does not exist then return false
    if (stat(taskfile.c_str(), &stInfo) != 0)
        return true;

    XMLTaskLoader loader;
    try{
        loader.load(taskfile);
    } catch(...){
        std::cout << "loader error" << std::endl;
        makeNewFile = true;
        return true;
    }
    makeNewFile = false;
    CartesianTask::Ptr ctask = loader.getCartesianTask();
    std::vector<CartesianTask::Ptr> allTasks = getAllTasks( ctask );
    //RW_WARN("");
    BOOST_FOREACH(CartesianTask::Ptr task, allTasks){
        BOOST_FOREACH(CartesianTarget::Ptr target, task->getTargets() ){
            int teststatus = target->getPropertyMap().get<int>("TestStatus", -1);
            if(teststatus<0)
                return true;
        }
    }
    return false;
}

int getNrOfTargets(std::string taskfile){
    struct stat stInfo;
    // if the file does not exist then return false
    if (stat(taskfile.c_str(), &stInfo) != 0)
        return 0;

    int nrTasks = 0;
    XMLTaskLoader loader;
    loader.load(taskfile);
    std::vector<CartesianTask::Ptr> allTasks = getAllTasks( loader.getCartesianTask() );
    //RW_WARN("");
    BOOST_FOREACH(CartesianTask::Ptr task, allTasks){
        nrTasks += task->getTargets().size();
    }
    return nrTasks;
}


