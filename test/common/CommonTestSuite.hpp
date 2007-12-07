#ifndef COMMONTESTSUITE_HPP
#define COMMONTESTSUITE_HPP

#include <boost/test/unit_test.hpp>

class CommonTestSuite : public boost::unit_test::test_suite{
    public:
        CommonTestSuite();
};

#endif // COMMONTESTSUITE_HPP
