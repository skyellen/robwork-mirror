#include <rw/models/WorkCell.hpp>
#include <rw/models/Models.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>

using rw::math::Q;
using namespace rw::models;
using rw::kinematics::State;
using namespace rw::pathplanning;
using rw::proximity::CollisionDetector;
using rw::loaders::PathLoader;
using rwlibs::proximitystrategies::ProximityStrategyYaobi;
using namespace rwlibs::pathplanners;

void plannerExample(WorkCell& workcell)
{
    // The common state for which to plan the paths.
    const State state = workcell.getDefaultState();

    // The first device of the workcell.
    Device::Ptr device = workcell.getDevices().front();

    CollisionDetector coldect(&workcell, ProximityStrategyYaobi::make());

    // The q constraint is to avoid collisions.
    QConstraint::Ptr constraint = QConstraint::make(&coldect, device, state);

    // the edge constraint tests the constraint on edges, eg. edge between two configurations
    QEdgeConstraintIncremental::Ptr edgeconstraint = QEdgeConstraintIncremental::makeDefault(
        constraint, device);

    // An SBL based point-to-point path planner.
    QToQPlanner::Ptr planner = SBLPlanner::makeQToQPlanner(
        SBLSetup::make(constraint, edgeconstraint, device));

    // A sampler of collision free configurations for the device.
    QSampler::Ptr cfreeQ = QSampler::makeConstrained(
        QSampler::makeUniform(device), constraint);

    // The start configuration for the path.
    Q pos = device->getQ(state);

    // Plan 10 paths to sampled collision free configurations.
    rw::trajectory::Path<Q> path;
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
