#ifndef TULTESTSUITE_HPP
#define TULTESTSUITE_HPP

#include <boost/test/unit_test.hpp>

class TULTestSuite : public boost::unit_test::test_suite{
    public:
        TULTestSuite();
};

#endif // TULTESTSUITE_HPP
