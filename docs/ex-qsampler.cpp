#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/math/Q.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyOpcode.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>
using namespace robwork;

void samplerExample(WorkCell& workcell)
{
    Device* device = workcell.getDevices().front();

    QConstraintPtr constraint = QConstraint::make(
        CollisionDetector::make(
            &workcell, ProximityStrategyOpcode::make()),
        device,
        workcell.getDefaultState());

    QSamplerPtr anyQ = QSampler::makeUniform(device);
    QSamplerPtr cfreeQ = QSampler::makeConstrained(anyQ, constraint);

    for (int i = 0; i < 4; i++) {
        const Q q = cfreeQ->sample();
        std::cout
            << "Q(" << i << ") is in collision: "
            << constraint->inCollision(q) << "\n";
    }
}
