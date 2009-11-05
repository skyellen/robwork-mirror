#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test_framework.hpp>

using boost::unit_test::test_suite;

test_suite* init_unit_test_suite(int argc, char* argv[])
{
    boost::unit_test::unit_test_log.set_threshold_level(boost::unit_test::log_messages );
    
    test_suite* test = BOOST_TEST_SUITE("RobWorkHardware test suite");
	// test->add(new MathTestSuite);
    return test;
}
