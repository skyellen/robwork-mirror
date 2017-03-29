/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/geometry/Box.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>

using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::proximity;

namespace {
class CollisionToleranceStrategyTest : public ::testing::TestWithParam<std::string> {
protected:
	CollisionToleranceStrategyTest():
		frameA(new MovableFrame("FrameA")),
		frameB(new MovableFrame("FrameB")),
		eps(0.000001),
		tolerance(0.01)
	{
	}

	void SetUp() {
		strategy = CollisionToleranceStrategy::Factory::makeStrategy(GetParam());
		ASSERT_FALSE(strategy.isNull());
		sstruct.addFrame(frameA);
		sstruct.addFrame(frameB);
		modelA = strategy->createModel();
		modelB = strategy->createModel();
		data.setCollisionQueryType(CollisionStrategy::AllContacts);
	}

	void TearDown() {
		// Important: models must be destroyed as some strategies must be able to cleanup internal caches.
		// PQP models are for instance cached based on the memory address of the GeometryData object
		// (other tests might create a different geometry on the same address)
		if (!strategy.isNull()) {
			strategy->destroyModel(modelA.get());
			strategy->destroyModel(modelB.get());
		}
	}

	CollisionToleranceStrategy::Ptr strategy;
	Frame* const frameA;
	Frame* const frameB;
	StateStructure sstruct;
	ProximityModel::Ptr modelA;
	ProximityModel::Ptr modelB;
	ProximityStrategyData data;
	const double eps;
	const double tolerance;
};

std::vector<std::string> strategies = CollisionToleranceStrategy::Factory::getStrategies();
}

TEST(Factory, CollisionToleranceStrategy) {
	EXPECT_GT(strategies.size(),0);
	for (std::size_t i = 0; i < strategies.size(); i++) {
		EXPECT_TRUE(CollisionToleranceStrategy::Factory::hasStrategy(strategies[i]));
		EXPECT_FALSE(CollisionToleranceStrategy::Factory::makeStrategy(strategies[i]).isNull());
	}
	EXPECT_FALSE(CollisionToleranceStrategy::Factory::hasStrategy("dfjgkjfgæah"));
	EXPECT_TRUE(CollisionToleranceStrategy::Factory::makeStrategy("dfjgkjfgæah").isNull());
}

INSTANTIATE_TEST_CASE_P(ProximityStrategy, CollisionToleranceStrategyTest, ::testing::ValuesIn(strategies));

TEST_P(CollisionToleranceStrategyTest, Plane_Cuboid) {
	static const double s = 0.1;
	static const double offset = 0.057;
	const rw::common::Ptr<Plane> plane = ownedPtr(new Plane());
	Geometry geomA(plane);
	const Box::Ptr box = ownedPtr(new Box(s,s,s));
	const Geometry::Ptr geomB = ownedPtr(new Geometry(box,Transform3D<>(Vector3D<>(0,0,offset))));

	static const Transform3D<> wTa = Transform3D<>::identity();

	// Test the interface where models are stored internally in the strategy
	strategy->addModel(frameA,geomA);
	strategy->addModel(frameB,geomB);
	EXPECT_FALSE(strategy->isWithinDistance(frameA,wTa,frameB,Transform3D<>(Vector3D<>::z()*(s/2-offset+tolerance+eps)),tolerance,data));
	EXPECT_TRUE(strategy->isWithinDistance(frameA,wTa,frameB,Transform3D<>(Vector3D<>::z()*(s/2-offset+tolerance-eps)),tolerance,data));
	strategy->clearFrames();

	// Test the interface where models are stored external to the strategy
	strategy->addGeometry(modelA.get(),geomA);
	strategy->addGeometry(modelB.get(),geomB);
	EXPECT_FALSE(strategy->isWithinDistance(modelA,wTa,modelB,Transform3D<>(Vector3D<>::z()*(s/2-offset+tolerance+eps)),tolerance,data));
	EXPECT_TRUE(strategy->isWithinDistance(modelA,wTa,modelB,Transform3D<>(Vector3D<>::z()*(s/2-offset+tolerance-eps)),tolerance,data));
}
