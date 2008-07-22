#include <rw/models/WorkCell.hpp>
#include <rw/models/Models.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QExpand.hpp>
#include <rw/pathplanning/QEdgeConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyOpcode.hpp>
#include <rwlibs/pathplanners/sbl/SBLQToQSamplerPlanner.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>
using namespace robwork;

#include <boost/foreach.hpp>

int main(int argc, char** argv)
{
    // Load the workcell named on the command line.
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <workcell>\n";
        return 1;
    }
    WorkCellPtr workcell = WorkCellLoader::load(argv[1]);

    // The number of paths to plan.
    const int maxCnt = 10;

    // The common state for which to plan the paths.
    const State state = workcell->getDefaultState();

    // The first device of the workcell.
    Device* device = workcell->getDevices().front();

    // A collision detector for a proximity strategy and a workcell.
    CollisionDetector detector(workcell, ProximityStrategyOpcode::make());

    // The configuration constraint for the path planning.
    QConstraintPtr constraint = QConstraint::make(&detector, device, state);

    // The edge constraint for the path planning.
    QEdgeConstraintPtr edge = QEdgeConstraint::makeDefault(constraint, device);

    // A sampler of configurations for the device.
    QSamplerPtr anyQ = QSampler::makeUniform(device);

    // An SBL based path planner.
    QToQPlannerPtr planner =
        QToQPlanner::make(
            SBLQToQSamplerPlanner::make(
                constraint,
                edge,
                QExpand::makeDecreasingUniformBox(
                    constraint,
                    device->getBounds(),
                    0.20)));

    // The start configuration for the path.
    Q pos = device->getQ(state);

    // Check that the start configuration does not collide.
    if (constraint->inCollision(pos)) {
        std::cout << "- Start configuration is in collision -\n";
        return 1;
    }

    // Plan maxCnt paths to random collision free configurations.
    Path path;
    for (int cnt = 0; cnt < maxCnt;) {
        const Q next = anyQ->sample();
        if (!constraint->inCollision(next)) {
            std::cout << "=> ";
            const bool ok = planner->query(pos, next, path);
            if (!ok) {
                std::cout << "- Path not found -\n";
                return 1;
            } else {
                std::cout << cnt << "\n";
                pos = next;
                ++cnt;
            }
        }
    }

    // Map the configurations to a sequence of states.
    const std::vector<State> states = Models::getStatePath(*device, path, state);

    // Write the sequence of states to a file.
    PathLoader::storeVelocityTimedStatePath(
        *workcell, states, "ex-path-planning.rwplay");
}
