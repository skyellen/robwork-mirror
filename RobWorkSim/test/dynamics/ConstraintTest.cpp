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

#include "../TestSuiteConfig.hpp"

#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/RigidObject.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/Constraint.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rwsim::dynamics;

BOOST_AUTO_TEST_CASE( ConstraintTest ) {
	BOOST_MESSAGE("- ConstraintTest");

	const RigidObject::Ptr objectA = ownedPtr(new RigidObject(new MovableFrame("BodyA")));
	const RigidObject::Ptr objectB = ownedPtr(new RigidObject(new MovableFrame("BodyB")));
	RigidBody bodyA(BodyInfo(),objectA);
	RigidBody bodyB(BodyInfo(),objectB);

	Constraint::Limit limit;
	limit.lowOn = true;
	limit.low = -1.0;

	const Transform3D<> pTc(Vector3D<>(-1.1,2.2,-3.3),EAA<>(0,-Pi,0).toRotation3D());

	{
		BOOST_MESSAGE("- - Fixed");
		Constraint c("Constraint",Constraint::Fixed,&bodyA,&bodyB);
		BOOST_CHECK(c.getType() == Constraint::Fixed);
		BOOST_CHECK_EQUAL(c.getDOF(), 0);
		BOOST_CHECK_EQUAL(c.getDOFLinear(), 0);
		BOOST_CHECK_EQUAL(c.getDOFAngular(), 0);
	}
	{
		BOOST_MESSAGE("- - Prismatic");
		Constraint c("Constraint",Constraint::Prismatic,&bodyA,&bodyB);
		BOOST_CHECK(c.getType() == Constraint::Prismatic);
		BOOST_CHECK_EQUAL(c.getDOF(), 1);
		BOOST_CHECK_EQUAL(c.getDOFLinear(), 1);
		BOOST_CHECK_EQUAL(c.getDOFAngular(), 0);

		// Check limits
		c.setLimit(0,limit);
		BOOST_CHECK(c.getLimit(0).high == limit.high);
		BOOST_CHECK(c.getLimit(0).low == limit.low);
		BOOST_CHECK(c.getLimit(0).highOn == limit.highOn);
		BOOST_CHECK(c.getLimit(0).lowOn == limit.lowOn);
	}
	{
		BOOST_MESSAGE("- - Revolute");
		Constraint c("Constraint",Constraint::Revolute,&bodyA,&bodyB);
		BOOST_CHECK(c.getType() == Constraint::Revolute);
		BOOST_CHECK_EQUAL(c.getDOF(), 1);
		BOOST_CHECK_EQUAL(c.getDOFLinear(), 0);
		BOOST_CHECK_EQUAL(c.getDOFAngular(), 1);

		// Check transform
		c.setTransform(pTc);
		BOOST_CHECK(c.getTransform() == pTc);
	}
	{
		BOOST_MESSAGE("- - Universal");
		Constraint c("Constraint",Constraint::Universal,&bodyA,&bodyB);
		BOOST_CHECK(c.getType() == Constraint::Universal);
		BOOST_CHECK_EQUAL(c.getDOF(), 2);
		BOOST_CHECK_EQUAL(c.getDOFLinear(), 0);
		BOOST_CHECK_EQUAL(c.getDOFAngular(), 2);

		// Check body pointers
		BOOST_CHECK(c.getBody1() == &bodyA);
		BOOST_CHECK(c.getBody2() == &bodyB);
	}
	{
		BOOST_MESSAGE("- - Spherical");
		Constraint c("Constraint",Constraint::Spherical,&bodyA,&bodyB);
		BOOST_CHECK(c.getType() == Constraint::Spherical);
		BOOST_CHECK_EQUAL(c.getDOF(), 3);
		BOOST_CHECK_EQUAL(c.getDOFLinear(), 0);
		BOOST_CHECK_EQUAL(c.getDOFAngular(), 3);
	}
	{
		BOOST_MESSAGE("- - Piston");
		Constraint c("Constraint",Constraint::Piston,&bodyA,&bodyB);
		BOOST_CHECK(c.getType() == Constraint::Piston);
		BOOST_CHECK_EQUAL(c.getDOF(), 2);
		BOOST_CHECK_EQUAL(c.getDOFLinear(), 1);
		BOOST_CHECK_EQUAL(c.getDOFAngular(), 1);
	}
	{
		BOOST_MESSAGE("- - PrismaticRotoid");
		Constraint c("Constraint",Constraint::PrismaticRotoid,&bodyA,&bodyB);
		BOOST_CHECK(c.getType() == Constraint::PrismaticRotoid);
		BOOST_CHECK_EQUAL(c.getDOF(), 2);
		BOOST_CHECK_EQUAL(c.getDOFLinear(), 1);
		BOOST_CHECK_EQUAL(c.getDOFAngular(), 1);

		// Check spring
		Constraint::SpringParams spring;
		spring.enabled = true;
		spring.compliance = Eigen::MatrixXd::Constant(2,2,1.234);
		spring.damping = Eigen::MatrixXd::Constant(2,2,4.321);
		c.setSpringParams(spring);
		BOOST_CHECK(c.getSpringParams().enabled);
		BOOST_CHECK_CLOSE(c.getSpringParams().compliance(1,0), 1.234, std::numeric_limits<double>::epsilon());
		BOOST_CHECK_CLOSE(c.getSpringParams().damping(0,1), 4.321, std::numeric_limits<double>::epsilon());
	}
	{
		BOOST_MESSAGE("- - PrismaticUniversal");
		Constraint c("Constraint",Constraint::PrismaticUniversal,&bodyA,&bodyB);
		BOOST_CHECK(c.getType() == Constraint::PrismaticUniversal);
		BOOST_CHECK_EQUAL(c.getDOF(), 3);
		BOOST_CHECK_EQUAL(c.getDOFLinear(), 1);
		BOOST_CHECK_EQUAL(c.getDOFAngular(), 2);

		// Check no spring
		BOOST_CHECK(!c.getSpringParams().enabled);
		// Check no limits
		BOOST_CHECK(!c.getLimit(0).highOn);
		BOOST_CHECK(!c.getLimit(0).lowOn);
		BOOST_CHECK(!c.getLimit(1).highOn);
		BOOST_CHECK(!c.getLimit(1).lowOn);
		BOOST_CHECK(!c.getLimit(2).highOn);
		BOOST_CHECK(!c.getLimit(2).lowOn);
	}
	{
		BOOST_MESSAGE("- - Free");
		Constraint c("Constraint",Constraint::Free,&bodyA,&bodyB);
		BOOST_CHECK(c.getType() == Constraint::Free);
		BOOST_CHECK_EQUAL(c.getDOF(), 6);
		BOOST_CHECK_EQUAL(c.getDOFLinear(), 3);
		BOOST_CHECK_EQUAL(c.getDOFAngular(), 3);

		// Check limits
		c.setLimit(5,limit);
		BOOST_CHECK(c.getLimit(5).high == limit.high);
		BOOST_CHECK(c.getLimit(5).low == limit.low);
		BOOST_CHECK(c.getLimit(5).highOn == limit.highOn);
		BOOST_CHECK(c.getLimit(5).lowOn == limit.lowOn);
	}
}
