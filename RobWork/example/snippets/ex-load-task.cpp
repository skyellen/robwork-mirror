#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/loader/TaskLoader.hpp>

#include <string>
#include <iostream>

using namespace rwlibs::task;

int main(int argc, char** argv) {
    if (argc == 2) {
        const std::string taskFile = argv[1];
        TaskLoader::Ptr taskloader = TaskLoader::Factory::getTaskLoader("xml");
        taskloader->load(taskFile);
        const TaskBase::Ptr task = taskloader->getTask();

        std::cout << "Task succesfully loaded. (1)\n";
    }

    else {
        std::cout << "Usage: " << argv[0] << " <workcell>? <task>\n";
    }

    return 0;
}
