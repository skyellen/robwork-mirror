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
