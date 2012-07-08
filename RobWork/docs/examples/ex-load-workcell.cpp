#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <string>
#include <iostream>

using namespace rw::models;
using namespace rw::loaders;



int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <workcell>\n";
        return 1;
    }

    const std::string file = argv[1];
    WorkCellPtr workcell = WorkCellLoader::load(file);

    std::cout << "Workcell " << *workcell << " successfully loaded.\n";
    return 0;
}
