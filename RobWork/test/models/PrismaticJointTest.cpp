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

#include <rw/models/PrismaticJoint.hpp>

using namespace rw::math;
using namespace rw::models;

BOOST_AUTO_TEST_CASE(PrismaticJointTest) {
	BOOST_MESSAGE("- Basic Implementation");
	{
		const Rotation3D<> rot = RPY<>(0,Pi/2,0).toRotation3D();
		const PrismaticJoint joint("Joint",Transform3D<>(Vector3D<>::z()*0.1,rot));
		const Transform3D<> wTp(Vector3D<>::x()*0.05,RPY<>(0,-Pi/4,0).toRotation3D());
		const Q q(1,0.2);

		// Check that the same result is achieved with pre- and post-multiplication:
		const Transform3D<> post = wTp*joint.getTransform(q[0]);
		Transform3D<> pre;
		joint.multiplyJointTransform(wTp,q,pre);
		BOOST_CHECK_SMALL((post.P()-pre.P()).normInf(),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(EAA<>(post.R()).angle()-EAA<>(pre.R()).angle(),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(cross(EAA<>(post.R()).axis(),EAA<>(pre.R()).axis()).normInf(),std::numeric_limits<double>::epsilon());

		// Check values are as expected
		const Rotation3D<> expRot = wTp.R()*rot;
		BOOST_CHECK_SMALL(EAA<>(post.R()).angle()-EAA<>(expRot).angle(),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(cross(EAA<>(post.R()).axis(),EAA<>(expRot).axis()).normInf(),std::numeric_limits<double>::epsilon());

		const Vector3D<> expPos = wTp.P()+wTp.R().getCol(2)*0.1+expRot.getCol(2)*q[0];
		BOOST_CHECK_SMALL((post.P()-expPos).normInf(),std::numeric_limits<double>::epsilon());
	}

	BOOST_MESSAGE("- Zero Rotation Implementation");
	{
		const PrismaticJoint joint("Joint",Transform3D<>(Vector3D<>::z()*0.1));
		const Transform3D<> wTp(Vector3D<>::x()*0.05,RPY<>(0,-Pi/4,0).toRotation3D());
		const Q q(1,0.2);

		// Check that the same result is achieved with pre- and post-multiplication:
		const Transform3D<> post = wTp*joint.getTransform(q[0]);
		Transform3D<> pre;
		joint.multiplyJointTransform(wTp,q,pre);
		BOOST_CHECK_SMALL((post.P()-pre.P()).normInf(),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(EAA<>(post.R()).angle()-EAA<>(pre.R()).angle(),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(cross(EAA<>(post.R()).axis(),EAA<>(pre.R()).axis()).normInf(),std::numeric_limits<double>::epsilon());

		// Check values are as expected
		const Rotation3D<>& expRot = wTp.R();
		BOOST_CHECK_SMALL(EAA<>(post.R()).angle()-EAA<>(expRot).angle(),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(cross(EAA<>(post.R()).axis(),EAA<>(expRot).axis()).normInf(),std::numeric_limits<double>::epsilon());

		const Vector3D<> expPos = wTp.P()+wTp.R().getCol(2)*(0.1+q[0]);
		BOOST_CHECK_SMALL((post.P()-expPos).normInf(),std::numeric_limits<double>::epsilon());
	}

	BOOST_MESSAGE("- Zero Translation Implementation");
	{
		const Rotation3D<> rot = RPY<>(0,Pi/2,0).toRotation3D();
		const PrismaticJoint joint("Joint",Transform3D<>(rot));
		const Transform3D<> wTp(Vector3D<>::x()*0.05,RPY<>(0,-Pi/4,0).toRotation3D());
		const Q q(1,0.2);

		// Check that the same result is achieved with pre- and post-multiplication:
		const Transform3D<> post = wTp*joint.getTransform(q[0]);
		Transform3D<> pre;
		joint.multiplyJointTransform(wTp,q,pre);
		BOOST_CHECK_SMALL((post.P()-pre.P()).normInf(),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(EAA<>(post.R()).angle()-EAA<>(pre.R()).angle(),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(cross(EAA<>(post.R()).axis(),EAA<>(pre.R()).axis()).normInf(),std::numeric_limits<double>::epsilon());

		// Check values are as expected
		const Rotation3D<> expRot = wTp.R()*rot;
		BOOST_CHECK_SMALL(EAA<>(post.R()).angle()-EAA<>(expRot).angle(),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(cross(EAA<>(post.R()).axis(),EAA<>(expRot).axis()).normInf(),std::numeric_limits<double>::epsilon());

		const Vector3D<> expPos = wTp.P()+expRot.getCol(2)*q[0];
		BOOST_CHECK_SMALL((post.P()-expPos).normInf(),std::numeric_limits<double>::epsilon());
	}
}
