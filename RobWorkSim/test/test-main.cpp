
//#define BOOST_TEST_MODULE "core-test"
#include "TestSuiteConfig.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

#include <rw/RobWork.hpp>

#ifdef RW_USE_BOOST_STATIC_TEST_LIBS
#include <boost/test/unit_test.hpp>
#else
#include <boost/test/included/unit_test.hpp>
#endif
using boost::unit_test::test_suite;

/**
 * @brief this struct will be alive for the complete duration of the test.
 * use constructor and destructor for initializing global variables
 */
struct InitRobWorkSim {
public:
    InitRobWorkSim(){ }
    ~InitRobWorkSim(){ }
};

BOOST_GLOBAL_FIXTURE(InitRobWorkSim);

std::string _testfilesDir;
std::string testFilePath() { return _testfilesDir; }

boost::unit_test::test_suite* init_unit_test_suite(int argc, char** const argv)
{
	rw::RobWork::init(argc,argv);

    // Get the top level suite from the registry
    boost::unit_test::unit_test_log.set_threshold_level(boost::unit_test::log_successful_tests);
    boost::property_tree::ptree tree;
    // load test configuration file

    //boost::filesystem::path full_path( boost::filesystem::current_path() );
    boost::filesystem::path path( argv[0] );
	boost::filesystem::path full_path = boost::filesystem::canonical(path);
    //std::ofstream log("/home/jimali/mylog.txt");
    //log << full_path << std::endl;
    //log << path << std::endl;
    //log << "sanfdælsajflojfjljdsifdsifidsfjd" << std::endl;
    //log << path.parent_path().string() + "/TestSuiteConfig.xml" << std::endl;
    rw::common::Log::getInstance()->setEnable(rw::common::Log::Debug);
    rw::common::Log::getInstance()->setLevel(rw::common::Log::Debug);
    rw::RobWork::init(argc,argv);
    //rw::RobWork::getInstance()->initialize();

    try{
        boost::property_tree::read_xml(full_path.parent_path().string() + "/TestSuiteConfig.xml", tree);
        //log << "TestConfig: " << full_path.parent_path().string() + "/TestSuiteConfig.xml" << std::endl;
        boost::property_tree::ptree &child = tree.get_child("RobWorkSimTest");
        _testfilesDir = child.get<std::string>("TestfilesDIR","testfiles") + "/";
        //log << "TESTDIR: " << _testfilesDir << std::endl;
        //log.close();
    } catch (...){
        return NULL;
    }
    // Return error code 1 if the one of test failed.
    return NULL;
}

