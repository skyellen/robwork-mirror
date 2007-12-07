#ifndef MATHTESTSUITE_HPP
#define MATHTESTSUITE_HPP

#include <boost/test/unit_test.hpp>

class MathTestSuite : public boost::unit_test::test_suite{
    public:
        MathTestSuite();
};

#endif // MATHTESTSUITE_HPP
