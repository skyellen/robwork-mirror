#include "CommonTestSuite.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include <string>

using namespace boost::unit_test;

void PropertyTest();
void LogTest();

CommonTestSuite::CommonTestSuite() :
    boost::unit_test::test_suite("CommonTestSuite")
{
    BOOST_MESSAGE("MathTestSuite");
    add( BOOST_TEST_CASE( &PropertyTest ) );
    add( BOOST_TEST_CASE( &LogTest ) );
}
