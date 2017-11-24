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

#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/models/ParallelLeg.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/RevoluteJoint.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

TEST(ParallelLeg, Common) {
	static const double lAB = 0.055;
	static const double lGaGb = 0.055;
	static const double lBFa = 0.015;
	static const double lFaFb = 0.020;
	static const double lFaD = lGaGb-lBFa;
	static const double angleFb = std::atan2(lAB,lFaD);
	static const double lFbFend = std::sqrt(lAB*lAB+lFaD*lFaD)-lFaFb;

	Frame* const base = new FixedFrame("Fixed",Transform3D<>::identity());
	Joint* const A = new RevoluteJoint("A",Transform3D<>(Vector3D<>::zero(),RPY<>(0,0,Pi/2)));
	Frame* const Aend = new FixedFrame("Aend",Transform3D<>::identity());
	Joint* const B = new RevoluteJoint("B",Transform3D<>(Vector3D<>(0,0,lAB),RPY<>(0,0,-Pi/2)));
	Frame* const Bend = new FixedFrame("Bend",Transform3D<>::identity());
	Joint* const Fa = new RevoluteJoint("Fa",Transform3D<>(Vector3D<>(lBFa,0,0),RPY<>(angleFb,0,0)));
	Joint* const Fb = new PrismaticJoint("Fb",Transform3D<>(Vector3D<>(lFaFb,0,0),RPY<>(0,Pi/2,0)));
	Frame* const Fend = new FixedFrame("Fend",Transform3D<>(Vector3D<>(0,0,lFbFend),RPY<>(0,-Pi/2,0)));

	std::vector<Frame*> frames(8);
	frames[0] = base;
	frames[1] = A;
	frames[2] = Aend;
	frames[3] = B;
	frames[4] = Bend;
	frames[5] = Fa;
	frames[6] = Fb;
	frames[7] = Fend;

	A->setActive(false);
	B->setActive(false);
	Fa->setActive(true);
	Fb->setActive(false);

	StateStructure sstruct;
	sstruct.addData(base);
	for (std::size_t i = 1; i < frames.size(); i++)
		sstruct.addFrame(frames[i],frames[i-1]);
	State state = sstruct.getDefaultState();

	ParallelLeg leg(frames);
	EXPECT_EQ(8,leg.getKinematicChain().size());
	EXPECT_EQ(base,leg.getBase());
	EXPECT_EQ(Fend,leg.getEnd());
	EXPECT_EQ(1,leg.nrOfActiveJoints());
	EXPECT_EQ(3,leg.nrOfPassiveJoints());
	EXPECT_EQ(4,leg.nrOfJoints());
	EXPECT_EQ(1,leg.getActuatedJoints().size());
	EXPECT_EQ(3,leg.getUnactuatedJoints().size());
	EXPECT_EQ(4,leg.getJointDOFs());
	EXPECT_EQ(4,leg.getQ(state).size());
	EXPECT_TRUE(leg.baseTend(state).equal(Transform3D<>(Vector3D<>(lAB,0,0),RPY<>(angleFb,0,0))));
	EXPECT_EQ(6,leg.baseJend(state).size1());
	EXPECT_EQ(4,leg.baseJend(state).size2());

	ASSERT_EQ(6,leg.baseJend(state).size1());
	ASSERT_EQ(4,leg.baseJend(state).size2());
	Eigen::Matrix<double, 6, 4> jacRef;
	jacRef << 0,  -lAB, -lAB, std::cos(angleFb),
			0,     lGaGb, lFaD, std::sin(angleFb),
			lGaGb, 0,     0,      0,
			0, 0, 0, 0,
			-1, 0, 0, 0,
			0, 1, 1, 0;
	EXPECT_TRUE((leg.baseJend(state).e()-jacRef).isZero(std::numeric_limits<double>::epsilon()));

	leg.setQ(Q(4,0.1,0.2,0.3,0.4),state);
	ASSERT_GE(4,leg.getQ(state).size());
	EXPECT_EQ(0.1,leg.getQ(state)[0]);
	EXPECT_EQ(0.2,leg.getQ(state)[1]);
	EXPECT_EQ(0.3,leg.getQ(state)[2]);
	EXPECT_EQ(0.4,leg.getQ(state)[3]);
	EXPECT_EQ(0.1,A->getData(state)[0]);
	EXPECT_EQ(0.2,A->getData(state)[1]);
	EXPECT_EQ(0.3,A->getData(state)[2]);
	EXPECT_EQ(0.4,A->getData(state)[3]);

	ASSERT_GE(1,leg.getActuatedJoints().size());
	EXPECT_EQ(Fa,leg.getActuatedJoints()[0]);
	ASSERT_GE(3,leg.getUnactuatedJoints().size());
	EXPECT_EQ(A,leg.getUnactuatedJoints()[0]);
	EXPECT_EQ(B,leg.getUnactuatedJoints()[1]);
	EXPECT_EQ(Fb,leg.getUnactuatedJoints()[2]);
}
