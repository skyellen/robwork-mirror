#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/math/Q.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rwlibs::proximitystrategies;

void samplerExample(WorkCell& workcell)
{
    Device::Ptr device = workcell.getDevices().front();

    CollisionDetector::Ptr coldect = ownedPtr( new CollisionDetector(&workcell, ProximityStrategyYaobi::make()) );

    QConstraint::Ptr constraint = QConstraint::make(
        coldect,
        device,
        workcell.getDefaultState());

    QSampler::Ptr anyQ = QSampler::makeUniform(device);
    QSampler::Ptr cfreeQ = QSampler::makeConstrained(anyQ, constraint);

    for (int i = 0; i < 4; i++) {
        const Q q = cfreeQ->sample();
        std::cout
            << "Q(" << i << ") is in collision: "
            << constraint->inCollision(q) << "\n";
    }
}
