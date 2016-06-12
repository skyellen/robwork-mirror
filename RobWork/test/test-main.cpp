
//#define BOOST_TEST_MODULE "core-test"
#include "TestSuiteConfig.hpp"

#include <rw/RobWork.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#include <iostream>

#ifdef RW_USE_BOOST_STATIC_TEST_LIBS
#include <boost/test/unit_test.hpp>
#else
#include <boost/test/included/unit_test.hpp>
#endif
//#if BOOST_VERSION >= 105900
//#include <boost/test/unit_test_parameters.hpp>
//#else
//#include <boost/test/detail/unit_test_parameters.hpp>
//#endif
using boost::unit_test::test_suite;

/**
 * @brief this struct will be alive for the complete duration of the test.
 * use constructor and destructor for initializing global variables
 */
struct InitRobWork {
public:
	InitRobWork(){
		//rw::RobWork::init();
	}
	~InitRobWork(){ }
};

BOOST_GLOBAL_FIXTURE(InitRobWork);

std::string _testfilesDir;
std::string testFilePath() { return _testfilesDir; }

boost::unit_test::test_suite* init_unit_test_suite(int argc, char** const argv)
{
    // Get the top level suite from the registry
    //boost::unit_test::unit_test_log.set_threshold_level(boost::unit_test::log_all_errors);
    //if( boost::unit_test::runtime_config::log_level() < boost::unit_test::log_messages )
    //boost::unit_test::unit_test_log.set_threshold_level(boost::unit_test::log_successful_tests);

    boost::property_tree::ptree tree;
    // load test configuration file

    boost::filesystem::path full_path( boost::filesystem::current_path() );
    boost::filesystem::path path( argv[0] );

    rw::RobWork::init(argc,(const char**)argv);

    try{
        boost::property_tree::read_xml(path.parent_path().string() + "/TestSuiteConfig.xml", tree);
        //log << "TestConfig: " << path.parent_path().string() + "/TestSuiteConfig.xml" << std::endl;
        boost::property_tree::ptree &child = tree.get_child("RobWorkTest");
        _testfilesDir = child.get<std::string>("TestfilesDIR","testfiles") + "/";
        //log << "TESTDIR: " << _testfilesDir << std::endl;
        //log.close();
    } catch (...){
        return NULL;
    }
    // Return error code 1 if the one of test failed.
    return NULL;
}

