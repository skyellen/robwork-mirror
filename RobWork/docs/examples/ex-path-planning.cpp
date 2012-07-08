#include <rw/models/WorkCell.hpp>
#include <rw/models/Models.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::pathplannning;
using namespace rw::proximity;
using namespace rw::loaders;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::pathplanners;

void plannerExample(WorkCell& workcell)
{
    // The common state for which to plan the paths.
    const State state = workcell.getDefaultState();

    // The first device of the workcell.
    DevicePtr device = workcell.getDevices().front();

    // The path planning constraint is to avoid collisions.
    const PlannerConstraint constraint = PlannerConstraint::make(
        ProximityStrategyYaobi::make(), &workcell, device, state);

    // An SBL based point-to-point path planner.
    QToQPlannerPtr planner = SBLPlanner::makeQToQPlanner(
        SBLSetup::make(constraint, device));

    // A sampler of collision free configurations for the device.
    QSamplerPtr cfreeQ = QSampler::makeConstrained(
        QSampler::makeUniform(device),
        constraint.getQConstraintPtr());

    // The start configuration for the path.
    Q pos = device->getQ(state);

    // Plan 10 paths to sampled collision free configurations.
    std::vector<Q> path;
    for (int cnt = 0; cnt < 10; cnt++) {
        const Q next = cfreeQ->sample();
        const bool ok = planner->query(pos, next, path);
        if (!ok) {
            std::cout << "Path " << cnt << " not found.\n";
            return;
        } else {
            pos = next;
        }
    }

    // Map the configurations to a sequence of states.
    const std::vector<State> states = Models::getStatePath(*device, path, state);

    // Write the sequence of states to a file.
    PathLoader::storeVelocityTimedStatePath(
        workcell, states, "ex-path-planning.rwplay");
}
