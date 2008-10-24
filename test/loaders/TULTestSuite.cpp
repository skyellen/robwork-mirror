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

#include "TULTestSuite.hpp"

#include "../TestSuiteConfig.h"
#include <string>

//#include <sandbox/loaders/>

using namespace boost::unit_test;

//using namespace rw::loaders;

void TULLoaderTest();
void PathLoaderTest();



void XMLRWLoaderTest(){

    /*BOOST_MESSAGE("XMLRWLoaderTestSuite");
    BOOST_MESSAGE("- Loading workcell file");
    std::auto_ptr<WorkCell> workcell =
        XMLRWLoader::load(testFilePath+"testscene.xml");

    BOOST_REQUIRE(NULL != workcell.get());
    BOOST_REQUIRE(workcell->getDevices().size() == 1);

    BOOST_MESSAGE("- Testing nr of devices");
    SerialDevice* device = (SerialDevice*)workcell->getDevices()[0];
    State state = workcell->getDefaultState();
*/

}

TULTestSuite::TULTestSuite() :
    boost::unit_test::test_suite("TULTestSuite")
{
    add( BOOST_TEST_CASE( &TULLoaderTest) );
    add( BOOST_TEST_CASE( &PathLoaderTest) );
    add( BOOST_TEST_CASE( &XMLRWLoaderTest) );
}
