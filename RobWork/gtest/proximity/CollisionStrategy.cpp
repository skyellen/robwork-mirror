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
#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>

using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::proximity;

namespace {
class CollisionStrategyTest : public ::testing::TestWithParam<std::string> {
protected:
	CollisionStrategyTest():
		frameA(new MovableFrame("FrameA")),
		frameB(new MovableFrame("FrameB")),
		eps(0.000001)
	{
	}

	void SetUp() {
		strategy = CollisionStrategy::Factory::makeStrategy(GetParam());
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

	CollisionStrategy::Ptr strategy;
	Frame* const frameA;
	Frame* const frameB;
	StateStructure sstruct;
	ProximityModel::Ptr modelA;
	ProximityModel::Ptr modelB;
	ProximityStrategyData data;
	const double eps;
};

std::vector<std::string> strategies = CollisionStrategy::Factory::getStrategies();
}

TEST(Factory, CollisionStrategy) {
	EXPECT_GT(strategies.size(),0);
	for (std::size_t i = 0; i < strategies.size(); i++) {
		EXPECT_TRUE(CollisionStrategy::Factory::hasStrategy(strategies[i]));
		EXPECT_FALSE(CollisionStrategy::Factory::makeStrategy(strategies[i]).isNull());
	}
	EXPECT_FALSE(CollisionStrategy::Factory::hasStrategy("dfjgkjfgæah"));
	EXPECT_TRUE(CollisionStrategy::Factory::makeStrategy("dfjgkjfgæah").isNull());
}

INSTANTIATE_TEST_CASE_P(ProximityStrategy, CollisionStrategyTest, ::testing::ValuesIn(strategies));

TEST_P(CollisionStrategyTest, Plane_Cuboid) {
	static const double s = 0.1;
	static const double offset = 0.057;
	const rw::common::Ptr<Plane> plane = ownedPtr(new Plane());
	Geometry geomA(plane);
	const Box::Ptr box = ownedPtr(new Box(s,s,s));
	const Geometry::Ptr geomB = ownedPtr(new Geometry(box,Transform3D<>(Vector3D<>(0,0,offset))));

	// Test the interface where models are stored internally in the strategy
	strategy->addModel(frameA,geomA);
	strategy->addModel(frameB,geomB);
	EXPECT_FALSE(strategy->inCollision(frameA,Transform3D<>::identity(),frameB,Transform3D<>(Vector3D<>::z()*(s/2-offset+eps)),data));
	EXPECT_TRUE(strategy->inCollision(frameA,Transform3D<>::identity(),frameB,Transform3D<>(Vector3D<>::z()*(s/2-offset-eps)),data));
	strategy->clearFrames();

	// Test the interface where models are stored external to the strategy
	strategy->addGeometry(modelA.get(),geomA);
	strategy->addGeometry(modelB.get(),geomB);
	EXPECT_FALSE(strategy->inCollision(modelA,Transform3D<>::identity(),modelB,Transform3D<>(Vector3D<>::z()*(s/2-offset+eps)),data));
	EXPECT_TRUE(strategy->inCollision(modelA,Transform3D<>::identity(),modelB,Transform3D<>(Vector3D<>::z()*(s/2-offset-eps)),data));

	// Check result
	const CollisionStrategy::Result& res = data.getCollisionData();
	EXPECT_EQ(modelA,res.a);
	EXPECT_EQ(modelB,res.b);
	EXPECT_EQ(0,res._aTb.P()[0]);
	EXPECT_EQ(0,res._aTb.P()[1]);
	EXPECT_FLOAT_EQ(static_cast<float>(s/2-offset-eps),static_cast<float>(res._aTb.P()[2]));
	EXPECT_TRUE(res._aTb.R().equal(Rotation3D<>::identity()));
	ASSERT_EQ(1,res._collisionPairs.size());
	EXPECT_EQ(0,res._collisionPairs[0].geoIdxA);
	EXPECT_EQ(0,res._collisionPairs[0].geoIdxB);
	EXPECT_EQ(0,res._collisionPairs[0].startIdx);
	EXPECT_EQ(res._geomPrimIds.size(),res._collisionPairs[0].size);
	//EXPECT_GT(res._geomPrimIds.size(),0);
	for (std::size_t i = 0; i < res._geomPrimIds.size(); i++) {
		EXPECT_GE(res._geomPrimIds[i].first,0);
		EXPECT_LT(res._geomPrimIds[i].first,2);
		EXPECT_GE(res._geomPrimIds[i].second,0);
		EXPECT_LT(res._geomPrimIds[i].second,12);
	}
	//EXPECT_GT(res.getNrPrimTests(),0);
	//EXPECT_GT(res.getNrBVTests(),0);
}

TEST_P(CollisionStrategyTest, Cuboid_Cuboid) {
	static const double s = 0.2;
	static const double offset = 0.057;
	const rw::common::Ptr<Box> box = ownedPtr(new Box(s, s, s));
	Geometry geomA(box);
	const Geometry::Ptr geomB = ownedPtr(new Geometry(box,Transform3D<>(Vector3D<>(0,0,offset))));

	// Test the interface where models are stored internally in the strategy
	strategy->addModel(frameA,geomA);
	strategy->addModel(frameB,geomB);
	EXPECT_TRUE(strategy->inCollision(frameA,Transform3D<>::identity(),frameB,Transform3D<>(Vector3D<>::z()*0.1),data));
	EXPECT_FALSE(strategy->inCollision(frameA,Transform3D<>::identity(),frameB,Transform3D<>(Vector3D<>::z()*10),data));
	EXPECT_FALSE(strategy->inCollision(frameA,Transform3D<>::identity(),frameB,Transform3D<>(Vector3D<>::z()*(s-offset+eps)),data));
	EXPECT_TRUE(strategy->inCollision(frameA,Transform3D<>::identity(),frameB,Transform3D<>(Vector3D<>::z()*(s-offset-eps)),data));
	strategy->clearFrames();

	// Test the interface where models are stored external to the strategy
	strategy->addGeometry(modelA.get(),geomA);
	strategy->addGeometry(modelB.get(),geomB);
	EXPECT_TRUE(strategy->inCollision(modelA,Transform3D<>::identity(),modelB,Transform3D<>(Vector3D<>::z()*0.1),data));
	EXPECT_FALSE(strategy->inCollision(modelA,Transform3D<>::identity(),modelB,Transform3D<>(Vector3D<>::z()*10),data));
	EXPECT_FALSE(strategy->inCollision(modelA,Transform3D<>::identity(),modelB,Transform3D<>(Vector3D<>::z()*(s-offset+eps)),data));
	EXPECT_TRUE(strategy->inCollision(modelA,Transform3D<>::identity(),modelB,Transform3D<>(Vector3D<>::z()*(s-offset-eps)),data));

	// Check result
	const CollisionStrategy::Result& res = data.getCollisionData();
	EXPECT_EQ(modelA,res.a);
	EXPECT_EQ(modelB,res.b);
	EXPECT_EQ(0,res._aTb.P()[0]);
	EXPECT_EQ(0,res._aTb.P()[1]);
	EXPECT_FLOAT_EQ(static_cast<float>(s-offset-eps),static_cast<float>(res._aTb.P()[2]));
	EXPECT_TRUE(res._aTb.R().equal(Rotation3D<>::identity()));
	ASSERT_EQ(1,res._collisionPairs.size());
	EXPECT_EQ(0,res._collisionPairs[0].geoIdxA);
	EXPECT_EQ(0,res._collisionPairs[0].geoIdxB);
	EXPECT_EQ(0,res._collisionPairs[0].startIdx);
	EXPECT_EQ(res._geomPrimIds.size(),res._collisionPairs[0].size);
	//EXPECT_GT(res._geomPrimIds.size(),0);
	for (std::size_t i = 0; i < res._geomPrimIds.size(); i++) {
		EXPECT_GE(res._geomPrimIds[i].first,0);
		EXPECT_LT(res._geomPrimIds[i].first,12);
		EXPECT_GE(res._geomPrimIds[i].second,0);
		EXPECT_LT(res._geomPrimIds[i].second,12);
	}
	//EXPECT_GT(res.getNrPrimTests(),0);
	//EXPECT_GT(res.getNrBVTests(),0);
}

TEST_P(CollisionStrategyTest, Cylinder_Cylinder) {
	static const double r = 0.12;
	static const float l = 0.2;
	static const double offset = 0.057;
	const rw::common::Ptr<Cylinder> cylinder = ownedPtr(new Cylinder(static_cast<float>(r), l));
	Geometry geomA(cylinder);
	const Geometry::Ptr geomB = ownedPtr(new Geometry(cylinder,Transform3D<>(Vector3D<>(offset,0,0))));

	// Test the interface where models are stored internally in the strategy
	strategy->addModel(frameA,geomA);
	strategy->addModel(frameB,geomB);
	EXPECT_TRUE(strategy->inCollision(frameA,Transform3D<>(Vector3D<>::x()*0.1),frameB,Transform3D<>::identity(),data));
	EXPECT_FALSE(strategy->inCollision(frameA,Transform3D<>(Vector3D<>::x()*10),frameB,Transform3D<>::identity(),data));
	EXPECT_FALSE(strategy->inCollision(frameA,Transform3D<>(Vector3D<>::x()*(r*2+offset+eps)),frameB,Transform3D<>::identity(),data));
	EXPECT_TRUE(strategy->inCollision(frameA,Transform3D<>(Vector3D<>::x()*(r*2+offset-eps)),frameB,Transform3D<>::identity(),data));
	strategy->clearFrames();

	// Test the interface where models are stored external to the strategy
	strategy->addGeometry(modelA.get(),geomA);
	strategy->addGeometry(modelB.get(),geomB);
	EXPECT_TRUE(strategy->inCollision(modelA,Transform3D<>(Vector3D<>::x()*0.1),modelB,Transform3D<>::identity(),data));
	EXPECT_FALSE(strategy->inCollision(modelA,Transform3D<>(Vector3D<>::x()*10),modelB,Transform3D<>::identity(),data));
	EXPECT_FALSE(strategy->inCollision(modelA,Transform3D<>(Vector3D<>::x()*(r*2+offset+eps)),modelB,Transform3D<>::identity(),data));
	EXPECT_TRUE(strategy->inCollision(modelA,Transform3D<>(Vector3D<>::x()*(r*2+offset-eps)),modelB,Transform3D<>::identity(),data));

	// Check result
	const CollisionStrategy::Result& res = data.getCollisionData();
	EXPECT_EQ(modelA,res.a);
	EXPECT_EQ(modelB,res.b);
	EXPECT_FLOAT_EQ(-r*2-offset+eps,res._aTb.P()[0]);
	EXPECT_EQ(0,res._aTb.P()[1]);
	EXPECT_EQ(0,res._aTb.P()[2]);
	EXPECT_TRUE(res._aTb.R().equal(Rotation3D<>::identity()));
	ASSERT_EQ(1,res._collisionPairs.size());
	EXPECT_EQ(0,res._collisionPairs[0].geoIdxA);
	EXPECT_EQ(0,res._collisionPairs[0].geoIdxB);
	EXPECT_EQ(0,res._collisionPairs[0].startIdx);
	EXPECT_EQ(res._geomPrimIds.size(),res._collisionPairs[0].size);
	//EXPECT_GT(res._geomPrimIds.size(),0);
	for (std::size_t i = 0; i < res._geomPrimIds.size(); i++) {
		EXPECT_GE(res._geomPrimIds[i].first,0);
		//EXPECT_LT(res._geomPrimIds[i].first,2);
		EXPECT_GE(res._geomPrimIds[i].second,0);
		//EXPECT_LT(res._geomPrimIds[i].second,12);
	}
	//EXPECT_GT(res.getNrPrimTests(),0);
	//EXPECT_GT(res.getNrBVTests(),0);
}
