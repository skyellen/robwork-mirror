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

#include <rw/kinematics/FixedFrame.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/ParallelDevice.hpp>
#include <rw/models/ParallelLeg.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/WorkCell.hpp>

using namespace rw::kinematics;
using rw::loaders::WorkCellLoader;
using namespace rw::math;
using namespace rw::models;

// Joint structure to test:
//
//  Fixed
//  |
//  A
//  |
//  |-Ga------------Gb---H
//  |                    |
//  |             __|    |
//  |          _Fb       |
//  |        _/          |
//  |      Fa            |
//  |      |
//  |-B---------------D
//            |       |
//            Ca      |
//            |       |
//            |       |
//            Cb   ---E
//
// Where all are revolute joints, except Fb which is prismatic
// Goal is that Fb/Gb remains connected, as well as D/H and Cb/E.
//
// Kinematic structure:
//                                      ________
//           /-------------------------|-Ca--Cb |
//          /                  ___     |        |
//     /-- B -----------------|-D-|----|-E      |
//    /    \     ________     |   |    |________|
//   /      \---|-Fa--Fb |    |   |
//  /           |        |    |   |
// A -----------|-Ga--Gb-|----|-H-|
//              |________|    |___|
//

namespace {
ParallelLeg* makeLeg(Frame* A, Frame* B, Frame* C = NULL, Frame* D = NULL, Frame* E = NULL, Frame* F = NULL) {
	std::vector<Frame*> chain(C==NULL?2:D==NULL?3:E==NULL?4:F==NULL?5:6);
	chain[0] = A;
	chain[1] = B;
	if (C != NULL)
		chain[2] = C;
	if (D != NULL)
		chain[3] = D;
	if (E != NULL)
		chain[4] = E;
	if (F != NULL)
		chain[5] = F;
	return new ParallelLeg(chain);
}
}

TEST(ParallelDevice, Junctions) {
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
	Joint* const Ca = new RevoluteJoint("Ca",Transform3D<>(Vector3D<>(0.030,0,0)));
	Joint* const Cb = new RevoluteJoint("Cb",Transform3D<>(Vector3D<>(0,-0.045,0)));
	Joint* const D = new RevoluteJoint("D",Transform3D<>(Vector3D<>(lGaGb,0,0)));
	Joint* const E = new RevoluteJoint("E",Transform3D<>(Vector3D<>(0,-0.045,0)));
	Frame* const Eend = new FixedFrame("Eend",Transform3D<>(Vector3D<>(-0.025,0,0)));
	Joint* const Fa = new RevoluteJoint("Fa",Transform3D<>(Vector3D<>(lBFa,0,0),RPY<>(angleFb,0,0)));
	Joint* const Fb = new PrismaticJoint("Fb",Transform3D<>(Vector3D<>(lFaFb,0,0),RPY<>(0,Pi/2,0)));
	Frame* const Fbend = new FixedFrame("Fbend",Transform3D<>(Vector3D<>(0,0,lFbFend),RPY<>(0,-Pi/2,0)));
	Joint* const Ga = new RevoluteJoint("Ga",Transform3D<>(Vector3D<>::zero(),RPY<>(0,0,-Pi/2)));
	Joint* const Gb = new RevoluteJoint("Gb",Transform3D<>(Vector3D<>(lGaGb,0,0),RPY<>(angleFb,0,0)));
	Joint* const H = new RevoluteJoint("H",Transform3D<>(Vector3D<>::zero(),RPY<>(-angleFb,0,0)));
	Frame* const Hend = new FixedFrame("Hend",Transform3D<>(Vector3D<>(0,-lAB,0)));

	A->setActive(true);
	B->setActive(false);
	Ca->setActive(false);
	Cb->setActive(false);
	D->setActive(false);
	E->setActive(false);
	Fa->setActive(false);
	Fb->setActive(true);
	Ga->setActive(false);
	Gb->setActive(false);
	H->setActive(false);

	StateStructure sstruct;
	sstruct.addData(base);
	sstruct.addFrame(A,base);
	sstruct.addFrame(Aend,A);
	sstruct.addFrame(Ga,Aend);
	sstruct.addFrame(Gb,Ga);
	sstruct.addFrame(H,Gb);
	sstruct.addFrame(Hend,H);
	sstruct.addFrame(B,Aend);
	sstruct.addFrame(Bend,B);
	sstruct.addFrame(D,Bend);
	sstruct.addFrame(E,D);
	sstruct.addFrame(Eend,E);
	sstruct.addFrame(Ca,Bend);
	sstruct.addFrame(Cb,Ca);
	sstruct.addFrame(Fa,Bend);
	sstruct.addFrame(Fb,Fa);
	sstruct.addFrame(Fbend,Fb);
	State state = sstruct.getDefaultState();

	std::vector<ParallelDevice::Legs> junctions(3);
	junctions[0].push_back(makeLeg(Aend,Ga,Gb));
	junctions[0].push_back(makeLeg(Aend,B,Bend,Fa,Fb,Fbend));
	junctions[1].push_back(makeLeg(Aend,Ga,Gb,H,Hend));
	junctions[1].push_back(makeLeg(Aend,B,Bend,D));
	junctions[2].push_back(makeLeg(Bend,Ca,Cb));
	junctions[2].push_back(makeLeg(Bend,D,E,Eend));

	ASSERT_TRUE(junctions[0][1]->baseTend(state).equal(junctions[0][0]->baseTend(state)));
	ASSERT_TRUE(junctions[1][1]->baseTend(state).equal(junctions[1][0]->baseTend(state)));
	ASSERT_TRUE(junctions[2][1]->baseTend(state).equal(junctions[2][0]->baseTend(state)));

	std::vector<Joint*> joints(11);
	joints[0] = A;
	joints[1] = B;
	joints[2] = Ca;
	joints[3] = Cb;
	joints[4] = D;
	joints[5] = E;
	joints[6] = Fa;
	joints[7] = Fb;
	joints[8] = Ga;
	joints[9] = Gb;
	joints[10] = H;

    const ParallelDevice* const device = new ParallelDevice("TestDevice", base, E, joints, state, junctions);
    const JointDevice* const jdevice = device;
    const Device* const ddevice = jdevice;

    // Generic Device functions
    EXPECT_EQ("TestDevice", ddevice->getName());
    EXPECT_TRUE(ddevice->baseTend(state).equal(Transform3D<>(Vector3D<>(lGaGb,-lAB-0.045,0))));
    EXPECT_TRUE(ddevice->baseTframe(H,state).equal(Transform3D<>(Vector3D<>(lGaGb,0,0))));
    EXPECT_TRUE(ddevice->worldTbase(state).equal(Transform3D<>::identity()));

    // Generic JointDevice functions
    EXPECT_EQ(base, jdevice->getBase());
    EXPECT_EQ(E, jdevice->getEnd());
    EXPECT_EQ(2, jdevice->getJoints().size());
    ASSERT_GE(2, jdevice->getJoints().size());
    EXPECT_EQ(A, jdevice->getJoints()[0]);
    EXPECT_EQ(Fb, jdevice->getJoints()[1]);
    EXPECT_EQ(2, jdevice->getDOF());
    EXPECT_EQ(2, jdevice->getBounds().first.size());
    EXPECT_EQ(2, jdevice->getBounds().second.size());
    EXPECT_EQ(2, jdevice->getVelocityLimits().size());
    EXPECT_EQ(2, jdevice->getAccelerationLimits().size());

    // ParallelDevice functions
    EXPECT_EQ(0, device->getLegs().size()); // There should be no legs when using the junction concept.
    EXPECT_EQ(2, device->getActiveJoints().size());
    ASSERT_GE(2, device->getActiveJoints().size());
    EXPECT_EQ(A, device->getActiveJoints()[0]);
    EXPECT_EQ(Fb, device->getActiveJoints()[1]);
    EXPECT_EQ(11, device->getAllJoints().size());
    EXPECT_EQ(11, device->getFullDOF());
    EXPECT_EQ(11, device->getAllBounds().first.size());
    EXPECT_EQ(11, device->getAllBounds().second.size());

    // Set full Q
    EXPECT_EQ(11, device->getFullQ(state).size());
    double vals[11] = {0,1,2,3,4,5,6,7,8,9,10};
    device->setFullQ(Q(11,vals),state);
    for (std::size_t i = 0; i <= 10; i++)
    	EXPECT_EQ(i, device->getFullQ(state)[i]);
    device->setFullQ(Q(11,0.),state);

    // Set Q
    EXPECT_EQ(2, device->getQ(state).size());
    device->setQ(Q(2,0.,0.015),state);
    const Q qTest = device->getFullQ(state); // save for later comparison
    {
    	static const double lFaFend = lFaFb+lFbFend+0.015;
    	static const double angleFaRef = std::acos((lFaD*lFaD+lFaFend*lFaFend-lAB*lAB)/(lFaD*lFaFend*2));
    	static const double angleDRef = std::acos((lFaD*lFaD+lAB*lAB-lFaFend*lFaFend)/(lFaD*lAB*2));
    	// Check full Q vector {A,B,Ca,Cb,D,E,Fa,Fb,Ga,Gb,H}
    	static const double eps = 1e-4;
    	EXPECT_NEAR(0,device->getFullQ(state)[0],eps); // A (controlled)
    	EXPECT_NEAR(angleDRef-Pi/2,device->getFullQ(state)[1],eps); // B
    	EXPECT_NEAR(-(angleDRef-Pi/2),device->getFullQ(state)[2],eps); // Ca
    	EXPECT_NEAR(angleDRef-Pi/2,device->getFullQ(state)[3],eps); // CB
    	EXPECT_NEAR(-(angleDRef-Pi/2),device->getFullQ(state)[4],eps); // D
    	EXPECT_NEAR(angleDRef-Pi/2,device->getFullQ(state)[5],eps); // E
    	EXPECT_NEAR(angleFaRef-angleFb,device->getFullQ(state)[6],eps); // Fa
    	EXPECT_NEAR(0.015,device->getFullQ(state)[7],eps); // Fb (controlled)
    	EXPECT_NEAR(angleDRef-Pi/2,device->getFullQ(state)[8],eps); // Ga
    	EXPECT_NEAR(angleFaRef-angleFb,device->getFullQ(state)[9],eps); // Gb
    	EXPECT_NEAR(-(angleDRef-Pi/2+angleFaRef-angleFb),device->getFullQ(state)[10],eps); // H

    	EXPECT_TRUE(junctions[0][1]->baseTend(state).equal(junctions[0][0]->baseTend(state),1e-6));
    	EXPECT_TRUE(junctions[1][1]->baseTend(state).equal(junctions[1][0]->baseTend(state),1e-6));
    	EXPECT_TRUE(junctions[2][1]->baseTend(state).equal(junctions[2][0]->baseTend(state),1e-6));
    }
    device->setQ(Q(2,0.,-0.015),state);
    {
    	static const double lFaFend = lFaFb+lFbFend-0.015;
    	static const double angleFaRef = std::acos((lFaD*lFaD+lFaFend*lFaFend-lAB*lAB)/(lFaD*lFaFend*2));
    	static const double angleDRef = std::acos((lFaD*lFaD+lAB*lAB-lFaFend*lFaFend)/(lFaD*lAB*2));
    	// Check full Q vector {A,B,Ca,Cb,D,E,Fa,Fb,Ga,Gb,H}
    	static const double eps = 1e-4;
    	EXPECT_NEAR(0,device->getFullQ(state)[0],eps); // A (controlled)
    	EXPECT_NEAR(angleDRef-Pi/2,device->getFullQ(state)[1],eps); // B
    	EXPECT_NEAR(-(angleDRef-Pi/2),device->getFullQ(state)[2],eps); // Ca
    	EXPECT_NEAR(angleDRef-Pi/2,device->getFullQ(state)[3],eps); // CB
    	EXPECT_NEAR(-(angleDRef-Pi/2),device->getFullQ(state)[4],eps); // D
    	EXPECT_NEAR(angleDRef-Pi/2,device->getFullQ(state)[5],eps); // E
    	EXPECT_NEAR(angleFaRef-angleFb,device->getFullQ(state)[6],eps); // Fa
    	EXPECT_NEAR(-0.015,device->getFullQ(state)[7],eps); // Fb (controlled)
    	EXPECT_NEAR(angleDRef-Pi/2,device->getFullQ(state)[8],eps); // Ga
    	EXPECT_NEAR(angleFaRef-angleFb,device->getFullQ(state)[9],eps); // Gb
    	EXPECT_NEAR(-(angleDRef-Pi/2+angleFaRef-angleFb),device->getFullQ(state)[10],eps); // H

    	EXPECT_TRUE(junctions[0][1]->baseTend(state).equal(junctions[0][0]->baseTend(state),1e-6));
    	EXPECT_TRUE(junctions[1][1]->baseTend(state).equal(junctions[1][0]->baseTend(state),1e-6));
    	EXPECT_TRUE(junctions[2][1]->baseTend(state).equal(junctions[2][0]->baseTend(state),1e-6));
    }

    delete device;

    // Test that we get same result with extra active joint, that is then disabled when setting setQ
	B->setActive(true);
	state = sstruct.getDefaultState();
    const ParallelDevice* const devExtraJoint = new ParallelDevice("TestDevice", base, E, joints, state, junctions);
    std::vector<bool> enabled(3,true);
    enabled[1] = false;
    devExtraJoint->setQ(Q(3,0.,0.2,0.015),enabled,state);
    EXPECT_TRUE((devExtraJoint->getFullQ(state).e()-qTest.e()).isZero(std::numeric_limits<double>::epsilon()));

    // Then test that we can control the other active joint instead
    enabled[1] = true;
    enabled[2] = false;
    devExtraJoint->setQ(Q(3,0.,0.2,0.015),enabled,state);
    {
    	static const double lFaFendRef = std::sqrt(lFaD*lFaD+lAB*lAB+lFaD*lAB*2*std::sin(0.2));
    	static const double angleFaRef = std::acos((lFaD+lAB*std::sin(0.2))/lFaFendRef);
    	// Check full Q vector {A,B,Ca,Cb,D,E,Fa,Fb,Ga,Gb,H}
    	static const double eps = 1e-4;
    	EXPECT_NEAR(0,devExtraJoint->getFullQ(state)[0],eps); // A (controlled)
    	EXPECT_NEAR(0.2,devExtraJoint->getFullQ(state)[1],eps); // B (controlled)
    	EXPECT_NEAR(-0.2,devExtraJoint->getFullQ(state)[2],eps); // Ca
    	EXPECT_NEAR(0.2,devExtraJoint->getFullQ(state)[3],eps); // CB
    	EXPECT_NEAR(-0.2,devExtraJoint->getFullQ(state)[4],eps); // D
    	EXPECT_NEAR(0.2,devExtraJoint->getFullQ(state)[5],eps); // E
    	EXPECT_NEAR(angleFaRef-angleFb,devExtraJoint->getFullQ(state)[6],eps); // Fa
    	EXPECT_NEAR(lFaFendRef-lFaFb-lFbFend,devExtraJoint->getFullQ(state)[7],eps); // Fb
    	EXPECT_NEAR(0.2,devExtraJoint->getFullQ(state)[8],eps); // Ga
    	EXPECT_NEAR(angleFaRef-angleFb,devExtraJoint->getFullQ(state)[9],eps); // Gb
    	EXPECT_NEAR(-(0.2+angleFaRef-angleFb),devExtraJoint->getFullQ(state)[10],eps); // H

    	EXPECT_TRUE(junctions[0][1]->baseTend(state).equal(junctions[0][0]->baseTend(state),1e-6));
    	EXPECT_TRUE(junctions[1][1]->baseTend(state).equal(junctions[1][0]->baseTend(state),1e-6));
    	EXPECT_TRUE(junctions[2][1]->baseTend(state).equal(junctions[2][0]->baseTend(state),1e-6));
    }

    delete devExtraJoint;

    for (std::size_t i = 0; i < junctions.size(); i++) {
    	for (std::size_t j = 0; j < junctions[i].size(); j++) {
    		delete junctions[i][j];
    	}
    }
}

namespace {
std::vector<Frame*> ParallelLegHEXAPOD(Frame* base, const std::string& scope, const Transform3D<double>& legBase, const Transform3D<double>& tool, double yaw, double pitch) {
	Joint* const joint1 = new RevoluteJoint(scope+".joint1",Transform3D<>::craigDH( 0, 0, 0, yaw));
	Joint* const joint2 = new RevoluteJoint(scope+".joint2",Transform3D<>::craigDH( Pi/2.0, 0, 0, Pi/2.0+pitch));
	Joint* const joint3 = new PrismaticJoint(scope+".joint3",Transform3D<>::craigDH( Pi/2.0, 0, 0, 0));
	Joint* const joint4 = new RevoluteJoint(scope+".joint4",Transform3D<>::craigDH( -Pi/2.0, 0, 0, Pi/2.0));
	Joint* const joint5 = new RevoluteJoint(scope+".joint5",Transform3D<>::craigDH( Pi/2.0, 0, 0, -Pi/2.0));
	Joint* const joint6 = new RevoluteJoint(scope+".joint6",Transform3D<>::craigDH( Pi/2.0, 0, 0, Pi/2.0));

	joint1->setActive(false);
	joint2->setActive(false);
	joint3->setActive(true);
	joint4->setActive(false);
	joint5->setActive(false);
	joint6->setActive(false);

    std::vector<Frame*> serialChain;
    serialChain.push_back(base);
    serialChain.push_back(new FixedFrame(scope+".legOffset",legBase));
    serialChain.push_back(joint1);
    serialChain.push_back(joint2);
    serialChain.push_back(joint3);
    serialChain.push_back(joint4);
    serialChain.push_back(joint5);
    serialChain.push_back(joint6);
    serialChain.push_back(new FixedFrame(scope+".Tool",tool));
    serialChain.push_back(new FixedFrame(scope+".End",Transform3D<>::identity()));

    return serialChain;
}
}

TEST(ParallelDevice, SerialChains) {
	Frame* const world = new FixedFrame("World", Transform3D<>::identity());

	const EAA<> R(normalize(Vector3D<>(1,0,1)),Pi);

	std::vector<std::vector<Frame*> > legChains(6);

	// Leg A
	const Transform3D<> baseTA(Vector3D<>(-60.206416539249155, -11.580306882912305, 0), R);
	const Transform3D<> toolTA(Vector3D<>(14.533675867691082, -8.586935664729255, 5.486283046170201), RPY<>(0,-0.5058297976189102, 0.2554802362083838));
	legChains[0] = ParallelLegHEXAPOD(world, "LegA", baseTA, toolTA,-0.2554802362083838, 0.5058297976189102);

	// Leg B
	const Transform3D<> baseTB(Vector3D<>(-40.132048213846446, -46.350132752361176,0), R);
	const Transform3D<> toolTB(Vector3D<>(-2.6407792002537804, 16.673016630261774, 5.486283046154413), RPY<>(0,-0.4486102166464917, 0.3501394675976276));
	legChains[1] = ParallelLegHEXAPOD(world, "LegB", baseTB, toolTB, -0.3501394675976276, 0.4486102166464917);

	// Leg C
	const Transform3D<> baseTC(Vector3D<>(40.13204821384643, -46.35013275236119,0), R);
	const Transform3D<> toolTC(Vector3D<>(2.6407792002537804, 16.673016630261774, 5.486283046154413), RPY<>(0,0.4486102166464917, 0.3501394675976276));
	legChains[2] = ParallelLegHEXAPOD(world, "LegC", baseTC, toolTC,-0.3501394675976276,-0.4486102166464917);

	// Leg D
	const Transform3D<> baseTD(Vector3D<>(60.20641653924915, -11.580306882912327, 0), R);
	const Transform3D<> toolTD(Vector3D<> (-14.533675867691082, -8.586935664729255, 5.486283046170201), RPY<>(0,0.5058297976189102, 0.2554802362083838));
	legChains[3] = ParallelLegHEXAPOD(world, "LegD", baseTD, toolTD,-0.2554802362083838, -0.5058297976189102);

	// Leg E
	const Transform3D<> baseTE(Vector3D<> (20.074368325402705, 57.930439635273515, 0), R);
	const Transform3D<> toolTE(Vector3D<> (-15.112667091527895, -7.521335766866318, 5.486283046171359), RPY<>(0,0.05084170375926413, -0.5595869360340129));
	legChains[4] = ParallelLegHEXAPOD(world, "LegE", baseTE, toolTE,0.5595869360340129, -0.05084170375926413);

	// Leg F
	const Transform3D<> baseTF(Vector3D<> (-20.0743683254027, 57.93043963527352, 0), R);
	const Transform3D<> toolTF(Vector3D<> (15.112667091527895, -7.521335766866318, 5.486283046171359), RPY<>(0,-0.05084170375926413, -0.5595869360340129));
	legChains[5] = ParallelLegHEXAPOD(world, "LegF", baseTF, toolTF,0.5595869360340129, 0.05084170375926413);

	StateStructure sstruct;
	sstruct.addData(world);
	for (std::size_t i = 0; i < legChains.size(); i++) {
		for (std::size_t j = 1; j < legChains[i].size(); j++) {
			sstruct.addFrame(legChains[i][j],legChains[i][j-1]);
		}
	}
	State state = sstruct.getDefaultState();

	const Q qh(6,0.,0.,92.6865,0.,0.,0.);

	std::vector<ParallelLeg*> legs(legChains.size());
	for (std::size_t i = 0; i < legChains.size(); i++) {
		legs[i] = new ParallelLeg(legChains[i]);
		legs[i]->setQ(qh,state);
	}

	const ParallelDevice hexapod(legs,"Hexapod",state);

	Q q = hexapod.getQ(state);

	hexapod.setQ(q,state);

	q = hexapod.getQ(state);
	EXPECT_NEAR(0,hexapod.baseTend(state).P()[0],5e-12);
	EXPECT_NEAR(0,hexapod.baseTend(state).P()[1],5e-12);
	EXPECT_NEAR(78.5,hexapod.baseTend(state).P()[2],1e-4);
	EXPECT_TRUE(hexapod.baseTend(state).R().equal(Rotation3D<>::identity(),1e-13));
	for (std::size_t i = 0; i < legs.size(); i++) {
		EXPECT_TRUE(legs[i]->baseTend(state).R().equal(hexapod.baseTend(state).R(),1e-12));
		EXPECT_NEAR(0,(legs[i]->baseTend(state).P()-hexapod.baseTend(state).P()).normInf(),1e-10);
	}

	q[0] += 1;
	hexapod.setQ(q,state);
	for (std::size_t i = 0; i < legs.size(); i++) {
		EXPECT_TRUE(legs[i]->baseTend(state).R().equal(hexapod.baseTend(state).R(),1e-12));
		EXPECT_NEAR(0,(legs[i]->baseTend(state).P()-hexapod.baseTend(state).P()).normInf(),1e-10);
	}

	q[1] += 2;
	q[2] += 1;
	q[3] += 3;

	hexapod.setQ(q,state);
	for (std::size_t i = 0; i < legs.size(); i++) {
		EXPECT_TRUE(legs[i]->baseTend(state).R().equal(hexapod.baseTend(state).R(),1e-10));
		EXPECT_NEAR(0,(legs[i]->baseTend(state).P()-hexapod.baseTend(state).P()).normInf(),5e-9);
	}

	q[1] += 2;
	q[2] += 1;
	q[3] += 3;
	q[4] += 20;
	q[5] += 3;

	hexapod.setQ(q,state);
	for (std::size_t i = 0; i < legs.size(); i++) {
		EXPECT_TRUE(legs[i]->baseTend(state).R().equal(hexapod.baseTend(state).R(),1e-11));
		EXPECT_NEAR(0,(legs[i]->baseTend(state).P()-hexapod.baseTend(state).P()).normInf(),1e-9);
	}

	EXPECT_EQ(6, q.size());
	ASSERT_GE(6, q.size());
	EXPECT_EQ(93.6865, q[0]);
	EXPECT_EQ(96.6865, q[1]);
	EXPECT_EQ(94.6865, q[2]);
	EXPECT_EQ(98.6865, q[3]);
	EXPECT_EQ(112.6865, q[4]);
	EXPECT_EQ(95.6865, q[5]);

	const Q qFull = hexapod.getFullQ(state);
	EXPECT_EQ(36, qFull.size());
	ASSERT_GE(36, qFull.size());
	EXPECT_EQ(93.6865, qFull[2]);
	EXPECT_EQ(96.6865, qFull[8]);
	EXPECT_EQ(94.6865, qFull[14]);
	EXPECT_EQ(98.6865, qFull[20]);
	EXPECT_EQ(112.6865, qFull[26]);
	EXPECT_EQ(95.6865, qFull[32]);
}

TEST(ParallelDevice, Robotiq) {
	const WorkCell::Ptr wc = WorkCellLoader::Factory::load(TestEnvironment::testfilesDir()+"devices/Robotiq-2-finger-85/robotiq.wc.xml");
	ASSERT_FALSE(wc.isNull());

	const ParallelDevice::Ptr robotiqDist = wc->findDevice<ParallelDevice>("RobotiqDistanceControl");
	const ParallelDevice::Ptr robotiqFinger = wc->findDevice<ParallelDevice>("RobotiqFingerControl");
	ASSERT_FALSE(robotiqDist.isNull());
	ASSERT_FALSE(robotiqFinger.isNull());

	State state = wc->getDefaultState();

    // Generic Device functions
    EXPECT_EQ("RobotiqDistanceControl", robotiqDist->getName());
    EXPECT_EQ("RobotiqFingerControl", robotiqFinger->getName());
    EXPECT_TRUE(robotiqDist->baseTend(state).equal(Transform3D<>(Vector3D<>(-0.0425,0.119008,0),RPY<>(0,Pi/2,0)),1e-6));
    EXPECT_TRUE(robotiqFinger->baseTend(state).equal(Transform3D<>(Vector3D<>(-0.0425,0.119008,0),RPY<>(0,Pi/2,0)),1e-4));

    // Generic JointDevice functions
    EXPECT_EQ("RobotiqDistanceControl.Base", robotiqDist->getBase()->getName());
    EXPECT_EQ("RobotiqDistanceControl.LeftMotorEnd", robotiqDist->getEnd()->getName());
    EXPECT_EQ("RobotiqFingerControl.Base", robotiqFinger->getBase()->getName());
    EXPECT_EQ("RobotiqFingerControl.LeftMotorEnd", robotiqFinger->getEnd()->getName());
    EXPECT_EQ(3, robotiqDist->getJoints().size());
    EXPECT_EQ(3, robotiqDist->getDOF());
    EXPECT_EQ(3, robotiqDist->getBounds().first.size());
    EXPECT_EQ(3, robotiqDist->getBounds().second.size());
    EXPECT_EQ(3, robotiqDist->getVelocityLimits().size());
    EXPECT_EQ(3, robotiqDist->getAccelerationLimits().size());
    EXPECT_EQ(4, robotiqFinger->getJoints().size()); // two motor joints maps to one DOF (dependent joints)
    EXPECT_EQ(3, robotiqFinger->getDOF());
    EXPECT_EQ(3, robotiqFinger->getBounds().first.size());
    EXPECT_EQ(3, robotiqFinger->getBounds().second.size());
    EXPECT_EQ(3, robotiqFinger->getVelocityLimits().size());
    EXPECT_EQ(3, robotiqFinger->getAccelerationLimits().size());

    // ParallelDevice functions
    EXPECT_EQ(0, robotiqDist->getLegs().size()); // There should be no legs when using the junction concept.
    EXPECT_EQ(3, robotiqDist->getJunctions().size());
    ASSERT_GE(robotiqDist->getJunctions().size(), 3);
    ASSERT_EQ(2, robotiqDist->getJunctions()[0].size());
    ASSERT_EQ(2, robotiqDist->getJunctions()[1].size());
    ASSERT_EQ(2, robotiqDist->getJunctions()[2].size());
    EXPECT_EQ(3, robotiqDist->getActiveJoints().size());
    EXPECT_EQ(13, robotiqDist->getAllJoints().size());
    EXPECT_EQ(12, robotiqDist->getFullDOF());
    EXPECT_EQ(12, robotiqDist->getAllBounds().first.size());
    EXPECT_EQ(12, robotiqDist->getAllBounds().second.size());
    EXPECT_EQ(0, robotiqFinger->getLegs().size()); // There should be no legs when using the junction concept.
    EXPECT_EQ(2, robotiqFinger->getJunctions().size());
    ASSERT_GE(robotiqFinger->getJunctions().size(), 2);
    ASSERT_EQ(2, robotiqFinger->getJunctions()[0].size());
    ASSERT_EQ(2, robotiqFinger->getJunctions()[1].size());
    EXPECT_EQ(4, robotiqFinger->getActiveJoints().size());
    EXPECT_EQ(10, robotiqFinger->getAllJoints().size());
    EXPECT_EQ(9, robotiqFinger->getFullDOF());
    EXPECT_EQ(9, robotiqFinger->getAllBounds().first.size());
    EXPECT_EQ(9, robotiqFinger->getAllBounds().second.size());

    ASSERT_EQ(3, robotiqDist->getQ(state).size());
    ASSERT_EQ(12, robotiqDist->getFullQ(state).size());
    ASSERT_EQ(3, robotiqFinger->getQ(state).size());
    ASSERT_EQ(9, robotiqFinger->getFullQ(state).size());

    // Set Q
    robotiqDist->setQ(Q(3,0.,0.04,0.),state);
    {
    	static const double eps = 1e-4;
    	EXPECT_NEAR(0.4506,robotiqDist->getFullQ(state)[0],eps);
    	EXPECT_NEAR(0.0008,robotiqDist->getFullQ(state)[1],eps);
    	EXPECT_NEAR(-0.4514,robotiqDist->getFullQ(state)[2],eps);
    	EXPECT_NEAR(0.4511,robotiqDist->getFullQ(state)[3],eps);
    	EXPECT_NEAR(-0.4511,robotiqDist->getFullQ(state)[4],eps);
    	EXPECT_NEAR(0.0008,robotiqDist->getFullQ(state)[5],eps);
    	EXPECT_NEAR(-0.4514,robotiqDist->getFullQ(state)[6],eps);
    	EXPECT_NEAR(0.4511,robotiqDist->getFullQ(state)[7],eps);
    	EXPECT_NEAR(-0.4511,robotiqDist->getFullQ(state)[8],eps);
    	EXPECT_EQ(0,robotiqDist->getFullQ(state)[9]);
    	EXPECT_EQ(0.04,robotiqDist->getFullQ(state)[10]);
    	EXPECT_EQ(0,robotiqDist->getFullQ(state)[11]);

    	EXPECT_TRUE(robotiqDist->getJunctions()[0][0]->baseTend(state).equal(robotiqDist->getJunctions()[0][1]->baseTend(state),1e-6));
    	EXPECT_TRUE(robotiqDist->getJunctions()[1][0]->baseTend(state).equal(robotiqDist->getJunctions()[1][1]->baseTend(state),1e-6));
    	EXPECT_TRUE(robotiqDist->getJunctions()[2][0]->baseTend(state).equal(robotiqDist->getJunctions()[2][1]->baseTend(state),1e-6));
    }

    robotiqDist->setQ(Q(3,-0.35,0.04,0.35),state);
    {
    	static const double eps = 1e-4;
    	EXPECT_NEAR(0.4858,robotiqDist->getFullQ(state)[0],eps);
    	EXPECT_NEAR(-0.2596,robotiqDist->getFullQ(state)[1],eps);
    	EXPECT_NEAR(0.1237,robotiqDist->getFullQ(state)[2],eps);
    	EXPECT_NEAR(0.2841,robotiqDist->getFullQ(state)[3],eps);
    	EXPECT_NEAR(0.0659,robotiqDist->getFullQ(state)[4],eps);
    	EXPECT_NEAR(-0.2596,robotiqDist->getFullQ(state)[5],eps);
    	EXPECT_NEAR(0.1237,robotiqDist->getFullQ(state)[6],eps);
    	EXPECT_NEAR(0.2841,robotiqDist->getFullQ(state)[7],eps);
    	EXPECT_NEAR(0.0658,robotiqDist->getFullQ(state)[8],eps);
    	EXPECT_EQ(-0.35,robotiqDist->getFullQ(state)[9]);
    	EXPECT_EQ(0.04,robotiqDist->getFullQ(state)[10]);
    	EXPECT_EQ(0.35,robotiqDist->getFullQ(state)[11]);

    	EXPECT_TRUE(robotiqDist->getJunctions()[0][0]->baseTend(state).equal(robotiqDist->getJunctions()[0][1]->baseTend(state),1e-6));
    	EXPECT_TRUE(robotiqDist->getJunctions()[1][0]->baseTend(state).equal(robotiqDist->getJunctions()[1][1]->baseTend(state),1e-6));
    	EXPECT_TRUE(robotiqDist->getJunctions()[2][0]->baseTend(state).equal(robotiqDist->getJunctions()[2][1]->baseTend(state),1e-6));
    }

    robotiqFinger->setQ(Q(3,0.35,0.25,0.25),state);
    {
    	static const double eps = 1e-4;
    	EXPECT_EQ(0.35,robotiqFinger->getFullQ(state)[0]);
    	EXPECT_NEAR(-0.1294,robotiqFinger->getFullQ(state)[1],eps);
    	EXPECT_NEAR(-0.0422,robotiqFinger->getFullQ(state)[2],eps);
    	EXPECT_EQ(0.25,robotiqFinger->getFullQ(state)[3]);
    	EXPECT_NEAR(-0.0716,robotiqFinger->getFullQ(state)[4],eps);
    	EXPECT_NEAR(-0.1294,robotiqFinger->getFullQ(state)[5],eps);
    	EXPECT_NEAR(-0.0422,robotiqFinger->getFullQ(state)[6],eps);
    	EXPECT_EQ(0.25,robotiqFinger->getFullQ(state)[7]);
    	EXPECT_NEAR(-0.0716,robotiqFinger->getFullQ(state)[8],eps);

    	EXPECT_TRUE(robotiqFinger->getJunctions()[0][0]->baseTend(state).equal(robotiqFinger->getJunctions()[0][1]->baseTend(state),1e-6));
    	EXPECT_TRUE(robotiqFinger->getJunctions()[1][0]->baseTend(state).equal(robotiqFinger->getJunctions()[1][1]->baseTend(state),1e-6));
    }

    robotiqFinger->setQ(Q(3,0.5,0.52,0.2),state);
    {
    	static const double eps = 1e-4;
    	EXPECT_EQ(0.5,robotiqFinger->getFullQ(state)[0]);
    	EXPECT_NEAR(0.0327,robotiqFinger->getFullQ(state)[1],eps);
    	EXPECT_NEAR(-0.5776,robotiqFinger->getFullQ(state)[2],eps);
    	EXPECT_EQ(0.52,robotiqFinger->getFullQ(state)[3]);
    	EXPECT_NEAR(-0.5650,robotiqFinger->getFullQ(state)[4],eps);
    	EXPECT_NEAR(-0.3623,robotiqFinger->getFullQ(state)[5],eps);
    	EXPECT_NEAR(0.3618,robotiqFinger->getFullQ(state)[6],eps);
    	EXPECT_EQ(0.2,robotiqFinger->getFullQ(state)[7]);
    	EXPECT_NEAR(0.2995,robotiqFinger->getFullQ(state)[8],eps);

    	EXPECT_TRUE(robotiqFinger->getJunctions()[0][0]->baseTend(state).equal(robotiqFinger->getJunctions()[0][1]->baseTend(state),1e-6));
    	EXPECT_TRUE(robotiqFinger->getJunctions()[1][0]->baseTend(state).equal(robotiqFinger->getJunctions()[1][1]->baseTend(state),1e-6));
    }
}
