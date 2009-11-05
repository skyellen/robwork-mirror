
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

#include <rw/loaders/TaskLoader.hpp>
#include <rw/task/Task.hpp>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

void printDefaultWorkCellStructure(const WorkCell& workcell);

int test_printDefaultWorkCellStructure(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "No file given.\n";
        return 1;
    }

    const std::string file = argv[1];
    WorkCellPtr wc = WorkCellLoader::load(file);

    printDefaultWorkCellStructure(*wc);

    return 0;
}

void visitTask(const Task& task);

int test_visitTask(int argc, char** argv)
{
    if (argc == 2) {
        const std::string taskFile = argv[1];
        const Task task = TaskLoader::load(taskFile, NULL);

        visitTask(task);
    }

    else {
        std::cout << "Usage: " << argv[0] << " <task>\n";
    }

    return 0;
}

void metricExample();
void printDeviceNames(const WorkCell& workcell);
void constraintExample(WorkCell& workcell);
void samplerExample(WorkCell& workcell);
void plannerExample(WorkCell& workcell);

int main(int argc, char** argv)
{
    // test_visitTask(argc, argv);

    // test_printDefaultWorkCellStructure(argc, argv);

    // metricExample();

    // printDeviceNames(*WorkCellLoader::load(argv[1]));

    // constraintExample(*WorkCellLoader::load(argv[1]));

    // samplerExample(*WorkCellLoader::load(argv[1]));
    plannerExample(*WorkCellLoader::load(argv[1]));
}
