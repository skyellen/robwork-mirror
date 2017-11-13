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

#include <rw/kinematics/StateStructure.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticUniversalJoint.hpp>
#include <rw/models/PrismaticSphericalJoint.hpp>
#include <rw/models/UniversalJoint.hpp>
#include <rw/models/SphericalJoint.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

namespace {
void testGenericJoint(Joint* const joint, const std::size_t dof) {
	EXPECT_EQ(dof,joint->getBounds().first.size());
	EXPECT_EQ(dof,joint->getBounds().second.size());
	joint->setBounds(std::make_pair(Q(dof,-0.6),Q(dof,0.2)));
	EXPECT_EQ(-0.6,joint->getBounds().first[dof-1]);
	EXPECT_EQ(0.2,joint->getBounds().second[dof-1]);

	EXPECT_EQ(dof,joint->getMaxVelocity().size());
	joint->setMaxVelocity(Q(dof,2.9));
	EXPECT_EQ(2.9,joint->getMaxVelocity()[dof-1]);

	EXPECT_EQ(dof,joint->getMaxAcceleration().size());
	joint->setMaxAcceleration(Q(dof,1.4));
	EXPECT_EQ(1.4,joint->getMaxAcceleration()[dof-1]);

	EXPECT_TRUE(joint->isActive());
	joint->setActive(false);
	EXPECT_FALSE(joint->isActive());
	joint->setActive(true);
	EXPECT_TRUE(joint->isActive());
}
void compareJointWithReferenceJoints(Joint* const joint, const std::vector<Joint*>& refJoints, const Rotation3D<>& offset = Rotation3D<>::identity(), const std::size_t offsetIndex = 0) {
	static const Transform3D<> T(Vector3D<>(1,2,3),RPY<>(45*Deg2Rad,45*Deg2Rad,45*Deg2Rad));

	const int dof = joint->getDOF();
	EXPECT_EQ(dof, joint->size());
	EXPECT_EQ("TestJoint", joint->getName());
	EXPECT_TRUE(joint->getFixedTransform().equal(Transform3D<>::identity()));
	joint->setFixedTransform(T);
	EXPECT_TRUE(joint->getFixedTransform().equal(T));

	StateStructure sstate;
	sstate.addData(joint); // StateStructure takes ownership of the joint
	for (std::size_t i = 0; i < refJoints.size(); i++)
		sstate.addData(refJoints[i]); // StateStructure takes ownership of the joint
	State state = sstate.getDefaultState();

	for (std::size_t i = 0; i < refJoints.size(); i++)
		EXPECT_EQ(0, joint->getData(state)[i]);

	double vals[4];
	for (std::size_t i = 0; i < refJoints.size(); i++)
		vals[i] = 0.1*i;
	joint->setData(state,vals);
	for (std::size_t i = 0; i < refJoints.size(); i++)
		refJoints[i]->setData(state,&vals[i]);

	for (std::size_t i = 0; i < refJoints.size(); i++)
		EXPECT_EQ(0.1*i, joint->getData(state)[i]);

	Transform3D<> revT;
	for (std::size_t i = 0; i < refJoints.size(); i++) {
		revT = revT*refJoints[i]->getTransform(state);
		if (i == offsetIndex)
			revT = revT*Transform3D<>(offset);
	}
	EXPECT_TRUE(joint->getJointTransform(state).R().equal(revT.R()));
	EXPECT_NEAR(0,(joint->getJointTransform(state).P()-revT.P()).normInf(),std::numeric_limits<double>::epsilon());
	const Transform3D<> revTfull = T*revT;
	EXPECT_TRUE(joint->getTransform(state).R().equal(revTfull.R()));
	EXPECT_NEAR(0,(joint->getTransform(state).P()-revTfull.P()).normInf(),std::numeric_limits<double>::epsilon());

	Transform3D<> multiplyTransform_res = T; // initial value should not influence result
	joint->multiplyTransform(T, state, multiplyTransform_res);
	EXPECT_TRUE(multiplyTransform_res.equal(T*joint->getTransform(state),1e-15));

	Jacobian jacobian(6+1,dof+2);
	Jacobian jacRev(6,dof);
	Transform3D<> bTj;

	jacobian = Jacobian::zero(6+1,dof+2);
	jacRev = Jacobian::zero(6,dof);
	bTj = Transform3D<>::identity();
	joint->getJacobian(1,2,joint->getJointTransform(state),Transform3D<>::identity(),state,jacobian);
	for (std::size_t i = 0; i < refJoints.size(); i++) {
		bTj = bTj*refJoints[i]->getTransform(state);
		refJoints[i]->getJacobian(0,i,bTj,Transform3D<>::identity(),state,jacRev);
		if (i == offsetIndex)
			bTj = bTj*Transform3D<>(offset);
	}
	EXPECT_TRUE((jacobian.e().block(1,2,6,dof)-jacRev.e()).isZero(std::numeric_limits<double>::epsilon()));

	jacobian = Jacobian::zero(6+1,dof+2);
	jacRev = Jacobian::zero(6,dof);
	bTj = Transform3D<>::identity();
	joint->getJacobian(1,2,joint->getJointTransform(state),T,state,jacobian);
	for (std::size_t i = 0; i < refJoints.size(); i++) {
		bTj = bTj*refJoints[i]->getTransform(state);
		refJoints[i]->getJacobian(0,i,bTj,T,state,jacRev);
		if (i == offsetIndex)
			bTj = bTj*Transform3D<>(offset);
	}
	EXPECT_TRUE((jacobian.e().block(1,2,6,dof)-jacRev.e()).isZero(1e-15));
}
}

TEST(Joint, Prismatic) {
	static const Transform3D<> T(Vector3D<>(1,2,3),RPY<>(10*Deg2Rad,20*Deg2Rad,30*Deg2Rad));
	PrismaticJoint* const joint = new PrismaticJoint("TestJoint", Transform3D<>::identity());

	EXPECT_EQ(1, joint->getDOF());
	EXPECT_EQ(1, joint->size());
	EXPECT_EQ("TestJoint", joint->getName());
	EXPECT_TRUE(joint->getFixedTransform().equal(Transform3D<>::identity()));
	joint->setFixedTransform(T);
	EXPECT_TRUE(joint->getFixedTransform().equal(T));

	StateStructure sstate;
	sstate.addData(joint); // StateStructure takes ownership of the joint
	State state = sstate.getDefaultState();

	EXPECT_EQ(0, joint->getData(state)[0]);

	double vals[1];
	vals[0] = 0.1;
	joint->setData(state,vals);

	EXPECT_EQ(0.1, joint->getData(state)[0]);

	EXPECT_TRUE(joint->getJointTransform(state).equal(Transform3D<>(Vector3D<>::z()*0.1)));
	EXPECT_TRUE(joint->getTransform(state).equal(Transform3D<>(T.P()+T.R().getCol(2)*0.1,T.R())));
	EXPECT_TRUE(joint->getJointTransform(state).equal(joint->getJointTransform(0.1)));
	EXPECT_TRUE(joint->getTransform(state).equal(joint->getTransform(0.1)));

	Transform3D<> multiplyTransform_res = T; // initial value should not influence result
	joint->multiplyTransform(T, state, multiplyTransform_res);
	EXPECT_TRUE(multiplyTransform_res.equal(T*joint->getTransform(state)));

	Transform3D<> multiplyJointTransform_res = T; // initial value should not influence result
	joint->multiplyJointTransform(T, Q(1,0.1), multiplyJointTransform_res);
	EXPECT_TRUE(multiplyJointTransform_res.equal(multiplyTransform_res));

	Jacobian jacobian(6+1,1+2);
	Jacobian jacRef = Jacobian::zero(6,1);

	jacobian = Jacobian::zero(6+1,1+2);
	jacRef(2,0) = 1;
	joint->getJacobian(1,2,joint->getJointTransform(state),Transform3D<>::identity(),state,jacobian);
	EXPECT_TRUE((jacobian.e().block(1,2,6,1)-jacRef.e()).isZero(std::numeric_limits<double>::epsilon()));

	jacobian = Jacobian::zero(6+1,1+2);
	jacRef(0,0) = T.R()(0,2);
	jacRef(1,0) = T.R()(1,2);
	jacRef(2,0) = T.R()(2,2);
	joint->getJacobian(1,2,joint->getTransform(state),Transform3D<>::identity(),state,jacobian);
	EXPECT_TRUE((jacobian.e().block(1,2,6,1)-jacRef.e()).isZero(std::numeric_limits<double>::epsilon()));
	jacobian = Jacobian::zero(6+1,1+2);
	joint->getJacobian(1,2,joint->getTransform(state),T,state,jacobian); // Using a transformation to the control point should not change anything
	EXPECT_TRUE((jacobian.e().block(1,2,6,1)-jacRef.e()).isZero(std::numeric_limits<double>::epsilon()));

	SCOPED_TRACE("Test of generic joint functions.");
	testGenericJoint(joint,1);
}

TEST(Joint, Revolute) {
	static const Transform3D<> T(Vector3D<>(1,2,3),RPY<>(10*Deg2Rad,20*Deg2Rad,30*Deg2Rad));
	RevoluteJoint* const joint = new RevoluteJoint("TestJoint", Transform3D<>::identity());

	EXPECT_EQ(1, joint->getDOF());
	EXPECT_EQ(1, joint->size());
	EXPECT_EQ("TestJoint", joint->getName());
	EXPECT_TRUE(joint->getFixedTransform().equal(Transform3D<>::identity()));
	joint->setFixedTransform(T);
	EXPECT_TRUE(joint->getFixedTransform().equal(T));

	StateStructure sstate;
	sstate.addData(joint); // StateStructure takes ownership of the joint
	State state = sstate.getDefaultState();

	EXPECT_EQ(0, joint->getData(state)[0]);

	double vals[1];
	vals[0] = Pi/2;
	joint->setData(state,vals);

	EXPECT_EQ(Pi/2, joint->getData(state)[0]);

	static const Rotation3D<> rotRef(0,-1,0,1,0,0,0,0,1);

	EXPECT_TRUE(joint->getJointTransform(state).equal(Transform3D<>(rotRef)));
	EXPECT_TRUE(joint->getTransform(state).equal(Transform3D<>(T.P(),T.R()*rotRef)));
	EXPECT_TRUE(joint->getJointTransform(state).equal(joint->getJointTransform(Pi/2)));
	EXPECT_TRUE(joint->getTransform(state).equal(joint->getTransform(Pi/2)));

	Transform3D<> multiplyTransform_res = T; // initial value should not influence result
	joint->multiplyTransform(T, state, multiplyTransform_res);
	EXPECT_TRUE(multiplyTransform_res.equal(T*joint->getTransform(state),1e-15));

	Transform3D<> multiplyJointTransform_res = T; // initial value should not influence result
	joint->multiplyJointTransform(T, Q(1,Pi/2), multiplyJointTransform_res);
	EXPECT_TRUE(multiplyJointTransform_res.equal(multiplyTransform_res));

	Jacobian jacobian(6+1,1+2);
	Jacobian jacRef(6,1);

	jacobian = Jacobian::zero(6+1,1+2);
	jacRef = Jacobian::zero(6,1);
	jacRef(5,0) = 1;
	joint->getJacobian(1,2,joint->getJointTransform(state),Transform3D<>::identity(),state,jacobian);
	EXPECT_TRUE((jacobian.e().block(1,2,6,1)-jacRef.e()).isZero(std::numeric_limits<double>::epsilon()));
	jacobian = Jacobian::zero(6+1,1+2);
	joint->getJacobian(1,2,joint->getJointTransform(state),Transform3D<>(Vector3D<>::z()*0.1),state,jacobian); // displacement of tcp along z should not change anything
	EXPECT_TRUE((jacobian.e().block(1,2,6,1)-jacRef.e()).isZero(std::numeric_limits<double>::epsilon()));
	jacobian = Jacobian::zero(6+1,1+2);
	jacRef(1,0) = 0.1; // if tcp is displaced in x, it will move in y when rotating around z.
	joint->getJacobian(1,2,joint->getJointTransform(state),Transform3D<>(Vector3D<>::x()*0.1),state,jacobian);
	EXPECT_TRUE((jacobian.e().block(1,2,6,1)-jacRef.e()).isZero(std::numeric_limits<double>::epsilon()));

	jacobian = Jacobian::zero(6+1,1+2);
	jacRef = Jacobian::zero(6,1);
	const Vector3D<> rotVec = T.R().getCol(2);
	jacRef(3,0) = rotVec[0];
	jacRef(4,0) = rotVec[1];
	jacRef(5,0) = rotVec[2];
	joint->getJacobian(1,2,joint->getTransform(state),T,state,jacobian); // Using same tcp frame gives zero change in positions
	EXPECT_TRUE((jacobian.e().block(1,2,6,1)-jacRef.e()).isZero(std::numeric_limits<double>::epsilon()));
	jacobian = Jacobian::zero(6+1,1+2);
	jacRef(0,0) = cross(rotVec,-T.P())[0];
	jacRef(1,0) = cross(rotVec,-T.P())[1];
	jacRef(2,0) = cross(rotVec,-T.P())[2];
	joint->getJacobian(1,2,joint->getTransform(state),Transform3D<>::identity(),state,jacobian);
	EXPECT_TRUE((jacobian.e().block(1,2,6,1)-jacRef.e()).isZero(std::numeric_limits<double>::epsilon()));

	SCOPED_TRACE("Test of generic joint functions.");
	testGenericJoint(joint,1);
}

TEST(Joint, Universal) {
	Joint* const joint = new UniversalJoint("TestJoint", Transform3D<>::identity());

	SCOPED_TRACE("Test of generic joint functions.");
	testGenericJoint(joint,2);

	std::vector<Joint*> refJoints(2);
	refJoints[0] = new RevoluteJoint("RevAJoint", Transform3D<>(Vector3D<>::zero(),RPY<>(0,Pi/2,0)));
	refJoints[1] = new RevoluteJoint("RevBJoint", Transform3D<>(Vector3D<>::zero(),RPY<>(0,0,-Pi/2)));
	static const Rotation3D<> revRend = RPY<>(-Pi/2,0,Pi/2).toRotation3D();

	EXPECT_EQ(2, joint->getDOF());
	SCOPED_TRACE("Comparison with reference joint sequence.");
	compareJointWithReferenceJoints(joint,refJoints,revRend,1);
}

TEST(Joint, PrismaticUniversal) {
	Joint* const joint = new PrismaticUniversalJoint("TestJoint", Transform3D<>::identity());

	SCOPED_TRACE("Test of generic joint functions.");
	testGenericJoint(joint,3);

	std::vector<Joint*> refJoints(3);
	refJoints[0] = new RevoluteJoint("RevAJoint", Transform3D<>(Vector3D<>::zero(),RPY<>(0,Pi/2,0)));
	refJoints[1] = new RevoluteJoint("RevBJoint", Transform3D<>(Vector3D<>::zero(),RPY<>(0,0,-Pi/2)));
	static const Rotation3D<> revRend = RPY<>(-Pi/2,0,Pi/2).toRotation3D();
	refJoints[2] = new PrismaticJoint("PrismaticJoint", Transform3D<>::identity());

	EXPECT_EQ(3, joint->getDOF());
	SCOPED_TRACE("Comparison with reference joint sequence.");
	compareJointWithReferenceJoints(joint,refJoints,revRend,1);
}

TEST(Joint, Spherical) {
	Joint* const joint = new SphericalJoint("TestJoint", Transform3D<>::identity());

	SCOPED_TRACE("Test of generic joint functions.");
	testGenericJoint(joint,3);

	std::vector<Joint*> refJoints(3);
	refJoints[0] = new RevoluteJoint("RevAJoint", Transform3D<>(Vector3D<>::zero(),RPY<>(0,Pi/2,0)));
	refJoints[1] = new RevoluteJoint("RevBJoint", Transform3D<>(Vector3D<>::zero(),RPY<>(0,0,-Pi/2)));
	refJoints[2] = new RevoluteJoint("RevCJoint", Transform3D<>(Vector3D<>::zero(),RPY<>(-Pi/2,0,Pi/2)));

	EXPECT_EQ(3, joint->getDOF());
	SCOPED_TRACE("Comparison with reference joint sequence.");
	compareJointWithReferenceJoints(joint,refJoints);
}

TEST(Joint, PrismaticSpherical) {
	Joint* const joint = new PrismaticSphericalJoint("TestJoint", Transform3D<>::identity());

	SCOPED_TRACE("Test of generic joint functions.");
	testGenericJoint(joint,4);

	std::vector<Joint*> refJoints(4);
	refJoints[0] = new RevoluteJoint("RevAJoint", Transform3D<>(Vector3D<>::zero(),RPY<>(0,Pi/2,0)));
	refJoints[1] = new RevoluteJoint("RevBJoint", Transform3D<>(Vector3D<>::zero(),RPY<>(0,0,-Pi/2)));
	refJoints[2] = new RevoluteJoint("RevCJoint", Transform3D<>(Vector3D<>::zero(),RPY<>(-Pi/2,0,Pi/2)));
	refJoints[3] = new PrismaticJoint("PrismaticJoint", Transform3D<>::identity());

	EXPECT_EQ(4, joint->getDOF());
	SCOPED_TRACE("Comparison with reference joint sequence.");
	compareJointWithReferenceJoints(joint,refJoints);
}
