#ifndef COLLISIONTESTSUITE_HPP
#define COLLISIONTESTSUITE_HPP

#include <boost/test/unit_test.hpp>

class CollisionTestSuite : public boost::unit_test::test_suite{
    public:
        CollisionTestSuite();
};

#endif // COLLISIONTESTSUITE_HPP
