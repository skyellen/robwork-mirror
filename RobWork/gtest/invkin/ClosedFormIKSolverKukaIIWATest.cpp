/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/invkin/ClosedFormIKSolverKukaIIWA.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/RevoluteJoint.hpp>

using rw::common::ownedPtr;
using rw::invkin::ClosedFormIKSolverKukaIIWA;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

namespace {
SerialDevice::Ptr getKukaIIWA(State& state) {
	Frame* const base = new FixedFrame("Base",Transform3D<>::identity());
	Joint* const joint1 = new RevoluteJoint("Joint1",Transform3D<>(Vector3D<>(0, 0,    0.158)));
	Joint* const joint2 = new RevoluteJoint("Joint2",Transform3D<>(Vector3D<>(0, 0,    0.182),RPY<>(0,0,-Pi/2.)));
	Joint* const joint3 = new RevoluteJoint("Joint3",Transform3D<>(Vector3D<>(0,-0.182,0),    RPY<>(0,0, Pi/2.)));
	Joint* const joint4 = new RevoluteJoint("Joint4",Transform3D<>(Vector3D<>(0, 0,    0.218),RPY<>(0,0, Pi/2.)));
	Joint* const joint5 = new RevoluteJoint("Joint5",Transform3D<>(Vector3D<>(0, 0.182,0),    RPY<>(0,0,-Pi/2.)));
	Joint* const joint6 = new RevoluteJoint("Joint6",Transform3D<>(Vector3D<>(0, 0,    0.218),RPY<>(0,0,-Pi/2.)));
	Joint* const joint7 = new RevoluteJoint("Joint7",Transform3D<>(Vector3D<>::zero(),        RPY<>(0,0, Pi/2.)));
	Frame* const end = new FixedFrame("TCP",Transform3D<>(Vector3D<>(0,0,0.126)));

	StateStructure stateStructure;
	stateStructure.addFrame(base);
	stateStructure.addFrame(joint1,base);
	stateStructure.addFrame(joint2,joint1);
	stateStructure.addFrame(joint3,joint2);
	stateStructure.addFrame(joint4,joint3);
	stateStructure.addFrame(joint5,joint4);
	stateStructure.addFrame(joint6,joint5);
	stateStructure.addFrame(joint7,joint6);
	stateStructure.addFrame(end,joint7);

	state = stateStructure.getDefaultState();

	const SerialDevice::Ptr device = ownedPtr(new SerialDevice(base,end,"KukaIIWA",state));
	std::pair<Q,Q> bounds;
	bounds.first = Q(7,-170*Deg2Rad,-120*Deg2Rad,-170*Deg2Rad,-120*Deg2Rad,-170*Deg2Rad,-120*Deg2Rad,-175*Deg2Rad);
	bounds.second = -bounds.first;
	device->setBounds(bounds);
	return device;
}
}

TEST(ClosedFormIKSolver, KukaIIWA) {
	State state;
	const SerialDevice::Ptr device = getKukaIIWA(state);
	ClosedFormIKSolverKukaIIWA solver(device,state);
	solver.setCheckJointLimits(true);

	std::vector<Q> qRefs(10);
	qRefs[0] = Q(7,  2.4,  1.4, -2.2,  2.0,  2.5, -1.2,  0.8);
	qRefs[1] = Q(7, -1.1, -1.7,  0.3, -0.9, -1.8,  0.2,  3.0);
	qRefs[2] = Q(7,  2.7,  2.1,  2.8,  2.0, -2.0,  0.9,  2.9);
	qRefs[3] = Q(7,  2.9,  1.9, -2.3, -0.1,  1.8,  1.3, -1.2);
	qRefs[4] = Q(7, -2.1, -2.1, -0.5, -1.6,  2.5,  0.6,  1.8);
	qRefs[5] = Q(7,  2.2,  1.9,  0.0,  0.7,  1.8, -1.9, -0.8);
	qRefs[6] = Q(7,  2.1, -1.2,  2.6,  0.8,  1.0, -0.4,  1.6);
	qRefs[7] = Q(7,  1.4,  1.0, -0.1, -0.5, -0.5,  0.7, -2.0);
	qRefs[8] = Q(7, -2.0, -0.8,  1.2,  1.2, -2.8, -0.8, -1.4);
	qRefs[9] = Q(7,  2.2, -1.9, -2.1, -1.7,  2.9,  1.4,  2.0);

	for (std::size_t refI = 0; refI < qRefs.size(); refI++)	{
		const Q& qRef = qRefs[refI];
		SCOPED_TRACE(qRef);
		device->setQ(qRef,state);
		const Transform3D<> T = device->baseTend(state);
		const std::vector<Q> solutions = solver.solve(T, state);
	    for (std::size_t i = 0; i < solutions.size(); i++) {
			device->setQ(solutions[i],state);
			const Transform3D<> Tfound = device->baseTend(state);
			EXPECT_NEAR(0,(Tfound.P()-T.P()).normInf(),1e-15);
			EXPECT_TRUE(T.R().equal(Tfound.R(),1e-15));
		}
	}
}
