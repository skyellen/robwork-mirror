#include "TULTestSuite.hpp"
#include <string>

using namespace boost::unit_test;

void TULLoaderTest();
void PathLoaderTest();

TULTestSuite::TULTestSuite() :
    boost::unit_test::test_suite("TULTestSuite")
{
    add( BOOST_TEST_CASE( &TULLoaderTest) );
    add( BOOST_TEST_CASE( &PathLoaderTest) );
}
