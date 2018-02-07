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

#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/models/RigidObject.hpp>
#include <rw/proximity/ProximityStrategy.hpp>

using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::proximity;

namespace {
class ProximityStrategyTest : public ::testing::TestWithParam<std::string> {
protected:
	void SetUp() {
		strategy = ProximityStrategy::Factory::makeStrategy(GetParam());
		ASSERT_FALSE(strategy.isNull());
	}

	ProximityStrategy::Ptr strategy;
};
std::vector<std::string> strategies = ProximityStrategy::Factory::getStrategies();
}

TEST(Factory, ProximityStrategy) {
	EXPECT_GT(strategies.size(),0);
	for (std::size_t i = 0; i < strategies.size(); i++) {
		EXPECT_TRUE(ProximityStrategy::Factory::hasStrategy(strategies[i]));
		EXPECT_FALSE(ProximityStrategy::Factory::makeStrategy(strategies[i]).isNull());
	}
	EXPECT_FALSE(ProximityStrategy::Factory::hasStrategy("dfjgkjfgæah"));
	EXPECT_TRUE(ProximityStrategy::Factory::makeStrategy("dfjgkjfgæah").isNull());
}

INSTANTIATE_TEST_CASE_P(ProximityStrategy, ProximityStrategyTest, ::testing::ValuesIn(strategies));

TEST_P(ProximityStrategyTest, Interface) {
	// Setup test structures
	std::vector<Frame*> frames;
	frames.push_back(new MovableFrame("Frame1"));
	frames.push_back(new MovableFrame("Frame2"));
	frames.push_back(new MovableFrame("Frame3"));
	StateStructure sstruct;
	sstruct.addFrame(frames[0]);
	sstruct.addFrame(frames[1]);
	sstruct.addFrame(frames[2]);

	const Geometry::Ptr geom1A = ownedPtr(new Geometry(ownedPtr(new Sphere(1)),"Geo1A"));
	const Geometry::Ptr geom1B = ownedPtr(new Geometry(ownedPtr(new Sphere(1)),"Geo1B"));
	const Geometry::Ptr geom1C = ownedPtr(new Geometry(ownedPtr(new Sphere(1)),"Geo1C"));
	const Geometry::Ptr geom2A = ownedPtr(new Geometry(ownedPtr(new Sphere(1)),"Geo2A"));
	const Geometry::Ptr geom2B = ownedPtr(new Geometry(ownedPtr(new Sphere(1)),"Geo2B"));
	const Geometry::Ptr geom3A = ownedPtr(new Geometry(ownedPtr(new Sphere(1)),"Geo3A"));
	const Geometry::Ptr geom3B = ownedPtr(new Geometry(ownedPtr(new Sphere(1)),"Geo3B"));
	geom1A->setFrame(frames[0]);
	geom1B->setFrame(frames[0]);
	geom1C->setFrame(frames[0]);
	geom2A->setFrame(frames[1]);
	geom2B->setFrame(frames[1]);
	geom3A->setFrame(frames[2]);
	geom3B->setFrame(frames[2]);

	std::vector<Geometry::Ptr> objectGeoms;
	objectGeoms.push_back(geom1B);
	objectGeoms.push_back(geom1C);
	objectGeoms.push_back(geom3B);
	const Object::Ptr object = ownedPtr(new RigidObject(frames,objectGeoms));

	// Test interface where ProximityModels are stored internally in the ProximityStrategy.
	EXPECT_FALSE(strategy->hasModel(frames[0]));
	strategy->addModel(frames[0],geom1A);
	EXPECT_TRUE(strategy->hasModel(frames[0]));
	strategy->clearFrame(frames[0]);
	EXPECT_FALSE(strategy->hasModel(frames[0]));
	strategy->addModel(frames[0],*geom1A);
	strategy->addModel(frames[1],*geom2A);
	strategy->clearFrames();
	EXPECT_FALSE(strategy->hasModel(frames[0]));
	EXPECT_FALSE(strategy->hasModel(frames[1]));
	EXPECT_TRUE(strategy->getModel(frames[0]).isNull());
	strategy->addModel(frames[0],*geom1A);
	EXPECT_FALSE(strategy->getModel(frames[0]).isNull());
	strategy->clearFrames();
	strategy->addModel(object);
	EXPECT_TRUE(strategy->hasModel(frames[0]));
	EXPECT_FALSE(strategy->hasModel(frames[1]));
	EXPECT_TRUE(strategy->hasModel(frames[2]));

	// Yaobi does not currently support the required interface for following tests..
	if (GetParam() == "Yaobi")
		return;

	const ProximityModel::Ptr model1 = strategy->getModel(frames[0]);
	const ProximityModel::Ptr model3 = strategy->getModel(frames[2]);
	//EXPECT_EQ(frames[0],model1->getFrame());
	//EXPECT_EQ(frames[2],model3->getFrame());
	ASSERT_NO_THROW(model1->getGeometryIDs());
	ASSERT_EQ(2,model1->getGeometryIDs().size());
	ASSERT_EQ(1,model3->getGeometryIDs().size());
	EXPECT_EQ("Geo1B",model1->getGeometryIDs()[0]);
	EXPECT_EQ("Geo1C",model1->getGeometryIDs()[1]);
	EXPECT_EQ("Geo3B",model3->getGeometryIDs()[0]);
	EXPECT_EQ(strategy.get(),model1->owner);
	EXPECT_EQ(strategy.get(),model3->owner);

	strategy->clearFrames();

	// Interface where ProximityModels are stored externally from the ProximityStrategy.
	const ProximityModel::Ptr modelA = strategy->createModel();
	const ProximityModel::Ptr modelB = strategy->createModel();
	EXPECT_FALSE(modelA.isNull());
	EXPECT_FALSE(modelB.isNull());

	// Raw access to model
	modelA->addGeometry(*geom1A);
	EXPECT_FALSE(strategy->hasModel(frames[0]));
	modelA->addGeometry(*geom1C);
	modelA->setFrame(frames[0]);

	EXPECT_EQ(frames[0],modelA->getFrame());
	ASSERT_EQ(2,modelA->getGeometryIDs().size());
	EXPECT_EQ("Geo1A",modelA->getGeometryIDs()[0]);
	EXPECT_EQ("Geo1C",modelA->getGeometryIDs()[1]);
	EXPECT_EQ(strategy.get(),modelA->owner);
	EXPECT_FALSE(modelA->removeGeometry("asdasd"));
	EXPECT_TRUE(modelA->removeGeometry("Geo1A"));
	ASSERT_EQ(1,modelA->getGeometryIDs().size());
	EXPECT_EQ("Geo1C",modelA->getGeometryIDs()[0]);

	// Proxy methods on strategy
	strategy->addGeometry(modelB.get(),geom3A);
	strategy->addGeometry(modelB.get(),*geom3B);
	EXPECT_FALSE(strategy->hasModel(frames[0]));
	EXPECT_EQ(NULL,modelB->getFrame());
	ASSERT_EQ(2,strategy->getGeometryIDs(modelB.get()).size());
	EXPECT_EQ("Geo3A",strategy->getGeometryIDs(modelB.get())[0]);
	EXPECT_EQ("Geo3B",strategy->getGeometryIDs(modelB.get())[1]);
	EXPECT_EQ(strategy.get(),modelB->owner);
	EXPECT_FALSE(strategy->removeGeometry(modelB.get(),"asdasd"));
	EXPECT_TRUE(strategy->removeGeometry(modelB.get(),"Geo3A"));
	ASSERT_EQ(1,strategy->getGeometryIDs(modelB.get()).size());
	EXPECT_EQ("Geo3B",strategy->getGeometryIDs(modelB.get())[0]);
	strategy->destroyModel(modelB.get());
	strategy->clear();
}
