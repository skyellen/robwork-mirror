#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

using rw::loaders::WorkCellFactory;
using rw::models::WorkCell;

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <workcell>\n";
        return 1;
    }

    const std::string file = argv[1];
    WorkCell::Ptr workcell = WorkCellFactory::load(file);

    std::cout << "Workcell " << *workcell << " successfully loaded.\n";
    return 0;
}
