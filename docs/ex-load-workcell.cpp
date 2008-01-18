#include <string>
#include <iostream>

#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " workcell-file\n";
        return 1;
    }

    const std::string file = argv[1];
    std::auto_ptr<WorkCell> workcell = WorkCellLoader::load(file);

    std::cout << "Workcell " << *workcell << " successfully loaded.\n";
    return 0;
}
