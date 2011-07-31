#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <string>
#include <iostream>

USE_ROBWORK_NAMESPACE
using namespace robwork;

int main(int argc, char** argv)
{
    if (argc == 2) {
        const std::string taskFile = argv[1];
        const Task task = XMLTaskLoader::load(taskFile, NULL);

        std::cout << "Task " << task << " succesfully loaded. (1)\n";
    }

    else if (argc == 3) {
        const std::string workcellFile = argv[1];
        WorkCellPtr workcell = WorkCellLoader::load(workcellFile);

        const std::string taskFile = argv[2];
        const Task task = XMLTaskLoader::load(taskFile, workcell);

        std::cout << "Task " << task << " succesfully loaded. (2)\n";
    }

    else {
        std::cout << "Usage: " << argv[0] << " <workcell>? <task>\n";
    }

    return 0;
}
