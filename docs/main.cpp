
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

void printDefaultWorkCellStructure(const WorkCell& workcell);

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "No file given.\n";
        return 1;
    }

    const std::string file = argv[1];
    std::auto_ptr<WorkCell> wc = WorkCellLoader::load(file);

    printDefaultWorkCellStructure(*wc);
}
