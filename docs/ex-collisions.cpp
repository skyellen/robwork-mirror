#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyOpcode.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>
using namespace robwork;

void collisionExample(WorkCellPtr workcell)
{
    CollisionDetector detector(
        workcell, ProximityStrategyOpcode::make());

    const bool collision =
        detector.inCollision(workcell->getDefaultState());

    std::cout
        << "Workcell "
        << *workcell
        << " is in collision in its initial state: "
        << collision
        << "\n";
}

int main(int argc, char** argv)
{
    if (argc == 2)
        collisionExample(WorkCellLoader::load(argv[1]));
    else
        std::cout << "Usage: " << argv[0] << " <workcell>\n";
}
