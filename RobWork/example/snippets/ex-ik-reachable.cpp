#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <boost/foreach.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rwlibs::proximitystrategies;

typedef std::vector<Transform3D<> > TransformPath;

TransformPath getRandomTargets(const Device& device, State state, int targetCnt)
{
    TransformPath result;
    QSampler::Ptr sampler = QSampler::makeUniform(device);
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
    QIKSampler::Ptr ik_any = QIKSampler::make(&device, state, NULL, NULL, 25);
    QIKSampler::Ptr ik_cfree = QIKSampler::makeConstrained(ik_any, &constraint, 25);

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

    WorkCell::Ptr workcell = WorkCellFactory::load(argv[1]);
    Device::Ptr device = workcell->getDevices().front();
    const State state = workcell->getDefaultState();

    CollisionDetector::Ptr detector = ownedPtr( new CollisionDetector(
        workcell, ProximityStrategyYaobi::make()) );

    QConstraint::Ptr constraint = QConstraint::make(detector, device, state);

    invkinExample(*device, state, *constraint);
}
