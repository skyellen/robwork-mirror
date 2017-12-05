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

#include "../TestEnvironment.hpp"

#include <rw/invkin/ParallelIKSolver.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/ParallelDevice.hpp>
#include <rw/models/ParallelLeg.hpp>
#include <rw/models/WorkCell.hpp>

using rw::invkin::ParallelIKSolver;
using namespace rw::kinematics;
using rw::loaders::WorkCellLoader;
using namespace rw::math;
using namespace rw::models;

TEST(ParallelDevice, Robotiq) {
	const WorkCell::Ptr wc = WorkCellLoader::Factory::load(TestEnvironment::testfilesDir()+"devices/Robotiq-2-finger-85/robotiq.wc.xml");
	ASSERT_FALSE(wc.isNull());

	const ParallelDevice::Ptr robotiq = wc->findDevice<ParallelDevice>("RobotiqFingerControl");
	ASSERT_FALSE(robotiq.isNull());

	const Frame* const fingerLeft = wc->findFrame(robotiq->getName()+"LeftIK");
	const Frame* const fingerRight = wc->findFrame(robotiq->getName()+"RightIK");
	//ASSERT_FALSE(rotationRefFrame == NULL);
	ASSERT_FALSE(fingerLeft == NULL);
	ASSERT_FALSE(fingerRight == NULL);

	const std::vector<ParallelDevice::Legs> junctions = robotiq->getJunctions();

	ParallelIKSolver solver(robotiq.get());

	VectorND<6, bool> enabled;
	enabled[0] = true;
	enabled[1] = false;
	enabled[2] = false;
	enabled[3] = false;
	enabled[4] = false;
	enabled[5] = true;

	// Solve for each finger individually and check that solutions are symmetric (that dependent joint is handled correctly)
	// First move left finger..
	{
		std::vector<ParallelIKSolver::Target> targets(1);
		targets[0] = ParallelIKSolver::Target(fingerLeft,Transform3D<>(Vector3D<>::x()*(-0.015),EAA<>(0,0,-0.5)),enabled);
		EXPECT_EQ(2,targets[0].dof());

		State state = wc->getDefaultState();
		const std::vector<Q> solutions = solver.solve(targets,state);
		EXPECT_EQ(1,solutions.size());
		ASSERT_GE(solutions.size(),1);

		EXPECT_NEAR(0.61176270099,solutions[0][0],1e-11);
		EXPECT_NEAR(0.31731028813,solutions[0][1],1e-11);
		EXPECT_NEAR(0.40223246809,solutions[0][2],1e-11);

		robotiq->setQ(solutions[0],state);

		EXPECT_EQ(solutions[0][0],robotiq->getQ(state)[0]);
		EXPECT_EQ(solutions[0][1],robotiq->getQ(state)[1]);
		EXPECT_EQ(solutions[0][2],robotiq->getQ(state)[2]);

		// Check that junctions are still connected
		EXPECT_TRUE(junctions[0][1]->baseTend(state).equal(junctions[0][0]->baseTend(state),1e-8));
		EXPECT_TRUE(junctions[1][1]->baseTend(state).equal(junctions[1][0]->baseTend(state),1e-8));

		// Check that target was reached
		EXPECT_NEAR(-0.015,robotiq->baseTframe(fingerLeft, state).P()[0],1e-7);
		EXPECT_NEAR(-0.5,EAA<>(robotiq->baseTframe(fingerLeft, state).R())[2],1e-6);
	}

	// .. then move right finger
	{
		std::vector<ParallelIKSolver::Target> targets(1);
		targets[0] = ParallelIKSolver::Target(fingerRight,Transform3D<>(Vector3D<>::x()*(0.015),EAA<>(0,0,0.5)),enabled);
		EXPECT_EQ(2,targets[0].dof());

		State state = wc->getDefaultState();
		const std::vector<Q> solutions = solver.solve(targets,state);
		EXPECT_EQ(1,solutions.size());
		ASSERT_GE(solutions.size(),1);

		EXPECT_NEAR(0.61176270099,solutions[0][0],1e-11);
		EXPECT_NEAR(0.40223246809,solutions[0][1],1e-11);
		EXPECT_NEAR(0.31731028813,solutions[0][2],1e-11);

		robotiq->setQ(solutions[0],state);

		EXPECT_EQ(solutions[0][0],robotiq->getQ(state)[0]);
		EXPECT_EQ(solutions[0][1],robotiq->getQ(state)[1]);
		EXPECT_EQ(solutions[0][2],robotiq->getQ(state)[2]);

		// Check that junctions are still connected
		EXPECT_TRUE(junctions[0][1]->baseTend(state).equal(junctions[0][0]->baseTend(state),1e-8));
		EXPECT_TRUE(junctions[1][1]->baseTend(state).equal(junctions[1][0]->baseTend(state),1e-8));

		// Check that target was reached
		EXPECT_NEAR(0.015,robotiq->baseTframe(fingerRight, state).P()[0],1e-7);
		EXPECT_NEAR(0.5,EAA<>(robotiq->baseTframe(fingerRight, state).R())[2],1e-6);
	}

	// Now try to move both fingers to reach the targets simultaneously
	{
		std::vector<ParallelIKSolver::Target> targets(2);
		targets[0] = ParallelIKSolver::Target(fingerLeft,Transform3D<>(Vector3D<>::x()*(-0.015),EAA<>(0,0,-0.5)),enabled);
		targets[1] = ParallelIKSolver::Target(fingerRight,Transform3D<>(Vector3D<>::x()*(0.015),EAA<>(0,0,0.5)),enabled);

		State state = wc->getDefaultState();
		const std::vector<Q> solutions = solver.solve(targets,state);
		EXPECT_EQ(1,solutions.size());
		ASSERT_GE(solutions.size(),1);

		EXPECT_NEAR(0.61176270099,solutions[0][0],1e-11);
		EXPECT_NEAR(0.31731028813,solutions[0][1],1e-11);
		EXPECT_NEAR(0.31731028813,solutions[0][2],1e-11);

		robotiq->setQ(solutions[0],state);

		EXPECT_EQ(solutions[0][0],robotiq->getQ(state)[0]);
		EXPECT_EQ(solutions[0][1],robotiq->getQ(state)[1]);
		EXPECT_EQ(solutions[0][2],robotiq->getQ(state)[2]);

		// Check that junctions are still connected
		EXPECT_TRUE(junctions[0][1]->baseTend(state).equal(junctions[0][0]->baseTend(state),1e-8));
		EXPECT_TRUE(junctions[1][1]->baseTend(state).equal(junctions[1][0]->baseTend(state),1e-8));

		// Check that target was reached
		EXPECT_NEAR(-0.015,robotiq->baseTframe(fingerLeft, state).P()[0],1e-7);
		EXPECT_NEAR(0.015,robotiq->baseTframe(fingerRight, state).P()[0],1e-7);
		EXPECT_NEAR(-0.5,EAA<>(robotiq->baseTframe(fingerLeft, state).R())[2],1e-6);
		EXPECT_NEAR(0.5,EAA<>(robotiq->baseTframe(fingerRight, state).R())[2],1e-6);
	}
}
