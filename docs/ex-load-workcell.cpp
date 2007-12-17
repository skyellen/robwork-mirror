#include <string>

#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

int main(int argc, char** argv)
{
    if (argc > 1) {
        const std::string file = argv[1];
        std::auto_ptr<WorkCell> workcell = WorkCellLoader::load(file);

        std::cout << "Work cell successfully loaded.\n";
        return 0;
    } else {
        std::cout << "No work cell loaded.\n";
        return 1;
    }
}
