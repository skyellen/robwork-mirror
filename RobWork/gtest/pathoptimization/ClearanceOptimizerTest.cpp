/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include <gtest/gtest.h>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/math/Metric.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/pathoptimization/clearance/ClearanceOptimizer.hpp>
#include <rwlibs/pathoptimization/clearance/MinimumClearanceCalculator.hpp>

using rw::common::ownedPtr;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using rw::trajectory::QPath;
using namespace rwlibs::pathoptimization;

namespace {
class TestClearanceCalculator: public ClearanceCalculator {
public:
	TestClearanceCalculator(const Device::CPtr& device): _device(device) {}
	~TestClearanceCalculator() {}
	double clearance(const State& state) const {
		const Q q = _device->getQ(state);
		const double d1 = q.norm2()-0.1; // distance to circle at origo
		const double d2 = (q-Q(2,0.,0.3)).norm2()-0.1; // distance to circle at {0,0.3}
		return std::min(d1,d2);
	}
private:
	const Device::CPtr _device;
};

void print(const std::string& name, const QPath& path) {
	std::cout << name << "={";
	for (std::size_t i = 0; i < path.size()-1; i++) {
		const Q& q = path[i];
		std::cout << "{" << q[0] << "," << q[1] << "},";
	}
	std::cout << "{" << path.back()[0] << "," << path.back()[1] << "}};" << std::endl;
}
}

TEST(ClearanceOptimizer, Test) {
	PrismaticJoint* const xJoint = new PrismaticJoint("xJoint",Transform3D<>(Vector3D<>::zero(),RPY<>(0,Pi/2,0)));
	PrismaticJoint* const yJoint = new PrismaticJoint("yJoint",Transform3D<>(Vector3D<>::zero(),RPY<>(0,0,-Pi/2)));
	StateStructure sstruct;
	sstruct.addFrame(xJoint);
	sstruct.addFrame(yJoint,xJoint);
	State state = sstruct.getDefaultState();

	const Device::Ptr device = ownedPtr(new SerialDevice(sstruct.getRoot(),yJoint,"TestDevice",state));
	device->setBounds(std::make_pair(Q(2,-0.5,-0.5),Q(2,0.5,0.5)));
	const ClearanceCalculator::CPtr clearanceCalculator = ownedPtr(new TestClearanceCalculator(device));
	const QMetric::CPtr metric = ownedPtr(new EuclideanMetric<Q>());

	ClearanceOptimizer optimizer(device, state, metric, clearanceCalculator);
	optimizer.setMinimumClearance(0.05);

	QPath inPath;
	inPath.push_back(Q(2,-0.5,0.));
	inPath.push_back(Q(2,-0.1,0.));
	inPath.push_back(Q(2,-0.1*std::sqrt(2.)/2,0.1*std::sqrt(2.)/2));
	inPath.push_back(Q(2, 0,  0.1));
	inPath.push_back(Q(2, 0.1*std::sqrt(2.)/2,0.1*std::sqrt(2.)/2));
	inPath.push_back(Q(2, 0.1,0.));
	inPath.push_back(Q(2, 0.5,0.));

	const double stepsize = 0.1;
	const std::size_t maxcount = 20;
	const double maxtime = 10;
	const QPath outPath = optimizer.optimize(inPath,stepsize,maxcount,maxtime);

	bool minimumClearanceReached = true;
	for (const Q& q : outPath) {
		device->setQ(q,state);
		const double clearance = clearanceCalculator->clearance(state);
		if (clearance < 0.05)
			minimumClearanceReached = false;
	}
	EXPECT_TRUE(minimumClearanceReached);

	//print("inPath",inPath);
	//print("outPath",outPath);
}
