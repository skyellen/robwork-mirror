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
