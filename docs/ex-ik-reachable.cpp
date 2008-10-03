#include <rw/pathplanning/QIKSampler.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/use_robwork_namespace.hpp>
#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

#include <boost/foreach.hpp>

typedef std::vector<Transform3D<> > TransformPath;

TransformPath getRandomTargets(const Device& device, State state, int targetCnt)
{
    TransformPath result;
    QSamplerPtr sampler = QSampler::makeUniform(device);
    for (int cnt = 0; cnt < targetCnt; cnt++) {
        device.setQ(sampler->sample(), state);
        result.push_back(device.baseTend(state));
    }
    return result;
}

void printReachableTargets(
    Device& device,
    const State& state,
    const TransformPath& targets,
    QIKSampler& ik)
{
    int i = 0;
    BOOST_FOREACH(const Transform3D<>& target, targets) {
        const Q q = ik.sample(target);
        std::cout << i << " " << (q.empty() ? "False" : "True") << "\n";
        ++i;
    }
}

void invkinExample(
    Device& device, const State& state, QConstraint& constraint)
{
    QIKSamplerPtr ik_any = QIKSampler::make(&device, state, NULL, NULL, 25);
    QIKSamplerPtr ik_cfree = QIKSampler::makeConstrained(ik_any, &constraint, 25);

    const TransformPath targets = getRandomTargets(device, state, 10);

    std::cout << "IK solutions found for targets:\n";
    printReachableTargets(device, state, targets, *ik_any);

    std::cout << "Collision free IK solutions found for targets:\n";
    printReachableTargets(device, state, targets, *ik_cfree);
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <workcell-file>\n";
        exit(1);
    }

    WorkCellPtr workcell = WorkCellLoader::load(argv[1]);
    DevicePtr device = workcell->getDevices().front();
    const State state = workcell->getDefaultState();

    CollisionDetectorPtr detector = CollisionDetector::make(
        workcell, ProximityStrategyYaobi::make());

    QConstraintPtr constraint = QConstraint::make(detector, device, state);

    invkinExample(*device, state, *constraint);
}
