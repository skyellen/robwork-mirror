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
    RevoluteJoint rjoint("RevoluteJointA",Transform3D<double>::identity());
    BOOST_CHECK(rjoint.getBounds().first < -1000000.0);
    BOOST_CHECK(rjoint.getBounds().second > 1000000.0);

    PrismaticJoint pjoint("PrismaticJointB",Transform3D<double>::identity());
    BOOST_CHECK(pjoint.getBounds().first < -1000000.0);
    BOOST_CHECK(pjoint.getBounds().second > 1000000.0);
}
void ModelsMessage(){
    BOOST_MESSAGE("ModelTestSuite");
}
ModelsTestSuite::ModelsTestSuite() :
    boost::unit_test::test_suite("ModelsTestSuite")
{
    add( BOOST_TEST_CASE( &ModelsMessage) );
    add( BOOST_TEST_CASE( &SerialDeviceTest) );
    //TODO: add( BOOST_TEST_CASE( &ParallelDeviceTest) );
    //TODO: add( BOOST_TEST_CASE( &ConveyorTest) );
}
