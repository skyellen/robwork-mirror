#include "CommonTestSuite.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include <string>

using namespace boost::unit_test;

void PropertyTest();
void LogTest();
void StringUtilTest();

void CommonMessage()
{
    BOOST_MESSAGE("CommonTestSuite");
    BOOST_CHECK(true); // To avoid a run-time warning.
}

CommonTestSuite::CommonTestSuite() :
    boost::unit_test::test_suite("CommonTestSuite")
{
    add(BOOST_TEST_CASE(&CommonMessage));
    add(BOOST_TEST_CASE(&PropertyTest));
    add(BOOST_TEST_CASE(&LogTest));
    add(BOOST_TEST_CASE(&StringUtilTest));
}
