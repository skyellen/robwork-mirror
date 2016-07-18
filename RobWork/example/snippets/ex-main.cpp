#include "ex-collisions.cpp"
#include "ex-constraints.cpp"
#include "ex-frame-to-frame-transform.cpp"
#include "ex-frame-to-frame-transforms.cpp"
#include "ex-get-path-planner.cpp"
#include "ex-get-state-path.cpp"
#include "ex-grip-frame.cpp"
#include "ex-ik.cpp"
#include "ex-is-daf.cpp"
#include "ex-metrics.cpp"
#include "ex-owned-ptr.cpp"
#include "ex-path-planning.cpp"
#include "ex-print-devices.cpp"
#include "ex-print-kinematic-tree.cpp"
#include "ex-qsampler.cpp"
#include "ex-world-transforms.cpp"

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

using namespace rw::kinematics;
using rw::models::WorkCell;
using rw::loaders::WorkCellLoader;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <workcell-file>\n";
        exit(1);
    }

    WorkCell::Ptr wc = WorkCellLoader::Factory::load(argv[1]);
	if (wc.isNull())
		RW_THROW("WorkCell could not be loaded.");

	const State defState = wc->getDefaultState();
    const Device::Ptr device = wc->getDevices().front();
	State state = defState;

	std::cout << "ex-collisions" << std::endl;
	collisionExample(*wc);

	std::cout << "ex-constraints" << std::endl;
	constraintExample(*wc);

	std::cout << "ex-frame-to-frame-transform" << std::endl;
	const Frame* frameA = wc->findFrame("FrameA");
	const Frame* frameB = wc->findFrame("FrameB");
	frameToFrameTransform(*frameA,*frameB,defState);

	std::cout << "ex-frame-to-frame-transforms" << std::endl;
	std::vector<State> states;
	const std::vector<Transform3D<> > transforms = frameToFrameTransforms(*frameA,*frameB,defState,states);

	std::cout << "ex-get-path-planner" << std::endl;
	CollisionStrategy::Ptr strategy;
	const PlannerConstraint constraint = PlannerConstraint::make(strategy,wc,device,defState);
	const QToQPlanner::Ptr planner = getQToQPlanner(device,constraint,"RRT");
	if (planner.isNull())
		RW_THROW("Could not create planner.");

	std::cout << "ex-get-state-path" << std::endl;
	const std::vector<Q> path;
	const std::vector<State> statePath = getStatePath(*device,path,defState);

	std::cout << "ex-grip-frame" << std::endl;
	MovableFrame* mframe = wc->findFrame<MovableFrame>("MFRAME");
	Frame* gripper = wc->findFrame("Gripper");
	gripMovableFrame(*mframe, *gripper, state);

	std::cout << "ex-ik" << std::endl;
	const Transform3D<> target;
	inverseKinematics(device, defState, target);

	std::cout << "ex-is-daf" << std::endl;
	const bool daf = isDaf(*gripper);
	std::cout << " - " << (daf?"True":"False") << std::endl;

	std::cout << "ex-metrics" << std::endl;
	metricExample();

	std::cout << "ex-owned-ptr" << std::endl;
	makeT();

	std::cout << "ex-path-planning" << std::endl;
	plannerExample(*wc);

	std::cout << "ex-print-devices" << std::endl;
	printDeviceNames(*wc);

	std::cout << "ex-print-kinematics-tree" << std::endl;
	printDefaultWorkCellStructure(*wc);

	std::cout << "ex-qsampler" << std::endl;
	samplerExample(*wc);

	std::cout << "ex-world-transforms" << std::endl;
	std::vector<Frame*> frames;
	const std::vector<Transform3D<> > worldTs = worldTransforms(frames, defState);

	return 0;
}
