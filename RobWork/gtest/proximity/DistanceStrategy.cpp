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
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/proximity/DistanceStrategy.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>

using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::proximity;

namespace {
void checkProximityStrategyData(const ProximityStrategyData& data, const DistanceStrategy::Result& res) {
	EXPECT_EQ(res.f1,data.getDistanceData().f1);
	EXPECT_EQ(res.f2,data.getDistanceData().f2);
	EXPECT_EQ(res.a,data.getDistanceData().a);
	EXPECT_EQ(res.b,data.getDistanceData().b);
	EXPECT_EQ(res.p1,data.getDistanceData().p1);
	EXPECT_EQ(res.p2,data.getDistanceData().p2);
	EXPECT_EQ(res.distance,data.getDistanceData().distance);
	EXPECT_EQ(res.geoIdxA,data.getDistanceData().geoIdxA);
	EXPECT_EQ(res.geoIdxB,data.getDistanceData().geoIdxB);
	EXPECT_EQ(res.idx1,data.getDistanceData().idx1);
	EXPECT_EQ(res.idx2,data.getDistanceData().idx2);
    EXPECT_EQ(0,data.rel_err);
	EXPECT_EQ(0,data.abs_err);
}

class DistanceStrategyTest : public ::testing::TestWithParam<std::string> {
protected:
	DistanceStrategyTest():
		frameA(new MovableFrame("FrameA")),
		frameB(new MovableFrame("FrameB")),
		tolerance(0.0005),
		eps(0.0001)
	{
	}

	void SetUp() {
		strategy = DistanceStrategy::Factory::makeStrategy(GetParam());
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

	DistanceStrategy::Ptr strategy;
	Frame* const frameA;
	Frame* const frameB;
	StateStructure sstruct;
	ProximityModel::Ptr modelA;
	ProximityModel::Ptr modelB;
	ProximityStrategyData data;
	const double tolerance;
	const double eps;
};

std::vector<std::string> strategies = DistanceStrategy::Factory::getStrategies();
}

TEST(Factory, DistanceStrategy) {
	EXPECT_GT(strategies.size(),0);
	for (std::size_t i = 0; i < strategies.size(); i++) {
		EXPECT_TRUE(DistanceStrategy::Factory::hasStrategy(strategies[i]));
		EXPECT_FALSE(DistanceStrategy::Factory::makeStrategy(strategies[i]).isNull());
	}
	EXPECT_FALSE(DistanceStrategy::Factory::hasStrategy("dfjgkjfgæah"));
	EXPECT_TRUE(DistanceStrategy::Factory::makeStrategy("dfjgkjfgæah").isNull());
}

INSTANTIATE_TEST_CASE_P(ProximityStrategy, DistanceStrategyTest, ::testing::ValuesIn(strategies));

TEST_P(DistanceStrategyTest, Plane_Triangle) {
	const rw::common::Ptr<Plane> plane = ownedPtr(new Plane());
	Geometry geomA(plane);
	const PlainTriMeshD::Ptr mesh = ownedPtr(new PlainTriMeshD());
	mesh->add(Triangle<>(Vector3D<>(0.3,-0.1,0),Vector3D<>(0.3+0.1,0,0),Vector3D<>(0.3,0.1,0)));
	//mesh->add(Triangle<>(Vector3D<>(0.3,-0.1,0),Vector3D<>(0.3+0.1,-0.1,-0.0001),Vector3D<>(0.3+0.1,0,0)));
	const Geometry::Ptr geomB = ownedPtr(new Geometry(mesh));

	static const Transform3D<> wTa = Transform3D<>::identity();
	static const Transform3D<> wTb(Vector3D<>::z()*(tolerance-eps));
	DistanceStrategy::Result res;

	// Test the interface where models are stored internally in the strategy
	strategy->addModel(frameA,geomA);
	strategy->addModel(frameB,geomB);
	res = strategy->distance(frameA,wTa,frameB,wTb,tolerance*1.5,data);
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.distance));
	res = strategy->distance(frameA,wTa,frameB,wTb,eps,data);
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.distance));
	res = strategy->distance(frameA,wTa,frameB,wTb,data);
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.distance));
	strategy->clearFrames();

	// Test the interface where models are stored external to the strategy
	strategy->addGeometry(modelA.get(),geomA);
	strategy->addGeometry(modelB.get(),geomB);
	res = strategy->distance(modelA,wTa,modelB,wTb,tolerance*1.5,data);
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.distance));
	res = strategy->distance(modelA,wTa,modelB,wTb,eps,data);
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.distance));
	res = strategy->distance(modelA,wTa,modelB,wTb,data);
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.distance));

	// Check result
	EXPECT_EQ(NULL,res.f1);
	EXPECT_EQ(NULL,res.f2);
	EXPECT_EQ(modelA,res.a);
	EXPECT_EQ(modelB,res.b);
	EXPECT_FLOAT_EQ(0.f,static_cast<float>(res.p1[2]));
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.p2[2]));
	EXPECT_EQ(0,res.geoIdxA); // only one geometry
	EXPECT_EQ(0,res.geoIdxB); // only one geometry
	EXPECT_GE(res.idx1,0u); // either first triangle on plane
	EXPECT_LE(res.idx1,1u); // ... or second
	EXPECT_EQ(0,res.idx2); // there is only one triangle

	SCOPED_TRACE("Check ProximityStrategyData");
	checkProximityStrategyData(data,res);
}

TEST_P(DistanceStrategyTest, Plane_Cuboid) {
	static const double s = 0.1;
	const rw::common::Ptr<Plane> plane = ownedPtr(new Plane());
	Geometry geomA(plane);
	const Box::Ptr box = ownedPtr(new Box(s,s,s));
	const Geometry::Ptr geomB = ownedPtr(new Geometry(box));

	static const Transform3D<> wTa = Transform3D<>::identity();
	static const Transform3D<> wTb(Vector3D<>::z()*(s/2+tolerance-eps));
	DistanceStrategy::Result res;

	// Test the interface where models are stored internally in the strategy
	strategy->addModel(frameA,geomA);
	strategy->addModel(frameB,geomB);
	res = strategy->distance(frameA,wTa,frameB,wTb,tolerance*1.5,data);
	EXPECT_NEAR(tolerance-eps,res.distance,std::numeric_limits<float>::epsilon());
	res = strategy->distance(frameA,wTa,frameB,wTb,eps,data);
	EXPECT_NEAR(tolerance-eps,res.distance,std::numeric_limits<float>::epsilon());
	res = strategy->distance(frameA,wTa,frameB,wTb,data);
	EXPECT_NEAR(tolerance-eps,res.distance,std::numeric_limits<float>::epsilon());
	strategy->clearFrames();

	// Test the interface where models are stored external to the strategy
	strategy->addGeometry(modelA.get(),geomA);
	strategy->addGeometry(modelB.get(),geomB);
	res = strategy->distance(modelA,wTa,modelB,wTb,tolerance*1.5,data);
	EXPECT_NEAR(tolerance-eps,res.distance,std::numeric_limits<float>::epsilon());
	res = strategy->distance(modelA,wTa,modelB,wTb,eps,data);
	EXPECT_NEAR(tolerance-eps,res.distance,std::numeric_limits<float>::epsilon());
	res = strategy->distance(modelA,wTa,modelB,wTb,data);
	EXPECT_NEAR(tolerance-eps,res.distance,std::numeric_limits<float>::epsilon());

	// Check result
	EXPECT_EQ(NULL,res.f1);
	EXPECT_EQ(NULL,res.f2);
	EXPECT_EQ(modelA,res.a);
	EXPECT_EQ(modelB,res.b);
	EXPECT_FLOAT_EQ(0.f,static_cast<float>(res.p1[2]));
	EXPECT_NEAR(tolerance-eps,res.p2[2],std::numeric_limits<float>::epsilon());
	EXPECT_EQ(0,res.geoIdxA); // only one geometry
	EXPECT_EQ(0,res.geoIdxB); // only one geometry
	EXPECT_GE(res.idx1,0u); // either first triangle on plane
	EXPECT_LE(res.idx1,1u); // ... or second
	EXPECT_GE(res.idx2,0u);
	EXPECT_LE(res.idx2,12u);

	SCOPED_TRACE("Check ProximityStrategyData");
	checkProximityStrategyData(data,res);
}
