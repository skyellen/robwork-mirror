#include "TrajectoryTestSuite.hpp"

using namespace boost::unit_test;

void PathTest();
//void SyncVelocityRampTest();

void StartMessage()
{
    BOOST_MESSAGE("TrajectoryTestSuite");
    BOOST_CHECK(true); // To avoid a run-time warning.
}

TrajectoryTestSuite::TrajectoryTestSuite() :
    boost::unit_test::test_suite("TrajectoryTestSuite")
{
    add(BOOST_TEST_CASE(&StartMessage));
    add(BOOST_TEST_CASE(&PathTest));
 //   add( BOOST_TEST_CASE( &SyncVelocityRampTest ) );
}
