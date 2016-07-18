#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>

using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;

void collisionExample(rw::models::WorkCell& workcell)
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
