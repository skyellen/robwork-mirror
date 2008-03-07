#include <rw/task/Task.hpp>
#include <rw/loaders/TaskLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

#include <string>
#include <iostream>

int main(int argc, char** argv)
{
    if (argc == 2) {
        const std::string taskFile = argv[1];
        const Task task = TaskLoader::load(taskFile, NULL);

        std::cout << "Task " << task << " succesfully loaded. (1)\n";
    }

    else if (argc == 3) {
        const std::string workcellFile = argv[1];
        std::auto_ptr<WorkCell> workcell = WorkCellLoader::load(workcellFile);

        const std::string taskFile = argv[2];
        const Task task = TaskLoader::load(taskFile, workcell.get());

        std::cout << "Task " << task << " succesfully loaded. (2)\n";
    }

    else {
        std::cout << "Usage: " << argv[0] << " <workcell>? <task>\n";
    }

    return 0;
}
