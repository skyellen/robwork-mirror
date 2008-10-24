/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "ModelsTestSuite.hpp"
#include <rw/models/SerialDevice.hpp>

#include <rw/kinematics/Frame.hpp>

#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>

#include <rw/math/EAA.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Rotation3D.hpp>
#include <memory>

#include <string>

using namespace boost::unit_test;

using namespace rw;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;

void SerialDeviceTest();
void WorkCellModelTest();
void ParallelDeviceTest();
void ConveyorTest();

void JointTest()
{
    std::auto_ptr<RevoluteJoint> rjoint(
        RevoluteJoint::make("RevoluteJointA", Transform3D<>::identity()));

    BOOST_CHECK(rjoint->getBounds().first < -1000000.0);
    BOOST_CHECK(rjoint->getBounds().second > 1000000.0);

	std::auto_ptr<PrismaticJoint> pjoint(
		PrismaticJoint::make("PrismaticJointB",Transform3D<>::identity()));
    BOOST_CHECK(pjoint->getBounds().first < -1000000.0);
    BOOST_CHECK(pjoint->getBounds().second > 1000000.0);
}

void ModelsMessage()
{
    BOOST_MESSAGE("ModelTestSuite");
    BOOST_CHECK(true); // To avoid a run-time warning.
}

ModelsTestSuite::ModelsTestSuite() :
    boost::unit_test::test_suite("ModelsTestSuite")
{
    add( BOOST_TEST_CASE( &ModelsMessage) );
    add( BOOST_TEST_CASE( &SerialDeviceTest) );
    //TODO: add( BOOST_TEST_CASE( &ParallelDeviceTest) );
    //TODO: add( BOOST_TEST_CASE( &ConveyorTest) );
}
