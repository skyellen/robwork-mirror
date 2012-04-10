#include <iostream>

#include <rw/math/Math.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/DistanceCalculator.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <sandbox/loaders/XercesXML/CollisionSetupLoader.hpp>
#include <rw/loaders/xml/XMLPathFormat.hpp>
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rwlibs::proximitystrategies;

int main(int argc, char** argv) {
	std::cout<<"Clearance Performance Test"<<std::endl;
	
	WorkCell::Ptr workcell = WorkCellLoader::load("D:/workspace/PickPlacePlanner/data/WorkCell/WorkCellRoboLab/PilotCellSimple.xml");
	if (workcell == NULL) {
		std::cout<<"Unable to load workcell"<<std::endl;
		return 0;
	}
	Device::Ptr device = workcell->findDevice("KukaKr30ha");
	if (device == NULL) {
		std::cout<<"Unable to find device"<<std::endl;
		return 0;
	}

	std::cout<<XMLPathFormat::TimedQId<<std::endl;


 /*   ProximityPairList strpairs = sandbox::CollisionSetupLoader::load("d:/workspace/PickPlacePlanner/data/WorkCell/ClearanceSetupPilot.xml");
	rw::kinematics::FramePairList clearanceTestFrames;
    for (ProximityPairList::iterator it = strpairs.begin(); it != strpairs.end(); ++it) {
        Frame* frame1 = workcell->findFrame((*it).first);
        Frame* frame2 = workcell->findFrame((*it).second);
		if (frame1 == NULL) 
			RW_THROW2(10001, "No frame named "<<(*it).first<<" found!");
		
		if (CollisionModelInfo::get(frame1).size() == 0)
			std::cout<<"No collision model to frame "<<(*it).first<<std::endl;

        if (frame2 == NULL) 
			RW_THROW2(10003, "No frame named "<<(*it).second<<" found!");
		
		if (CollisionModelInfo::get(frame1).size() == 0)
			std::cout<<"No collision model to frame "<<(*it).second<<std::endl;
		clearanceTestFrames.push_back(std::make_pair(frame1, frame2));
    }
	std::cout<<"Clearance Test Frames = "<<clearanceTestFrames.size()<<std::endl;
 
	rw::common::Ptr<ProximityStrategyPQP> proximityStrategy = ownedPtr(new ProximityStrategyPQP());
	DistanceCalculator::Ptr distanceCalculator = ownedPtr(new DistanceCalculator(clearanceTestFrames, proximityStrategy));

	Math::seed(0);
	QSampler::Ptr sampler = QSampler::makeUniform(device);
	State state = workcell->getDefaultState();
	//For loading the geometries
	distanceCalculator->distance(state);
	distanceCalculator->resetComputationTimeAndCount();
	int collisions = 0;
	double sum = 0;
	for (size_t i = 0; i<1000; i++) {
		Q q = sampler->sample();
		device->setQ(q, state);
		double d = distanceCalculator->distance(state).distance;
		if (d == 0)
			collisions++;
		sum += d;
	}
	std::cout<<"Time (s) = "<<distanceCalculator->getComputationTime()<<std::endl;
	std::cout<<"Avg Time (ms) = "<<1000.*distanceCalculator->getComputationTime() / distanceCalculator->getCount()<<std::endl;
	std::cout<<"Count = "<<distanceCalculator->getCount()<<std::endl;
	std::cout<<"Number of collisions = "<<collisions<<std::endl;
	std::cout<<"Total Clearance Dist = "<<sum<<std::endl;
*/

	return 0;
}