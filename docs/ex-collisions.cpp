#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>
using namespace robwork;

void collisionExample(WorkCell& workcell)
{
    CollisionDetector detector(
        &workcell, ProximityStrategyYaobi::make());

    const bool collision =
        detector.inCollision(workcell.getDefaultState());

    std::cout
        << "Workcell "
        << workcell
        << " is in collision in its initial state: "
        << collision
        << "\n";
}
