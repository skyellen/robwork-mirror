
//#define BOOST_TEST_MODULE "core-test"
#include <boost/test/included/unit_test.hpp>

using boost::unit_test::test_suite;

/**
 * @brief this struct will be alive for the complete duration of the test.
 * use constructor and destructor for initializing global variables
 */
struct InitRobWork {
public:
	InitRobWork(){ }
	~InitRobWork(){ }
};

#if BOOST_VERSION >= 105900
BOOST_GLOBAL_FIXTURE(InitRobWork);
#else
BOOST_GLOBAL_FIXTURE(InitRobWork) // the semicolon was included in macro in Boost 1.58 and earlier (to avoid pedantic warnings about extra ';')
#endif

boost::unit_test::test_suite* init_unit_test_suite(int argc, char** const argv)
{
    // Get the top level suite from the registry
    //boost::unit_test::unit_test_log.set_threshold_level(boost::unit_test::log_test_suites);

    // Return error code 1 if the one of test failed.
    return 0;
}

