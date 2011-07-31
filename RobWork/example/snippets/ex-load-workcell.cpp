#include <rw/rw.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <workcell>\n";
        return 1;
    }

    const std::string file = argv[1];
    WorkCell::Ptr workcell = WorkCellLoader::load(file);

    std::cout << "Workcell " << *workcell << " successfully loaded.\n";
    return 0;
}
