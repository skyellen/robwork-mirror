#ifndef COLLISIONTESTSUITE_HPP
#define COLLISIONTESTSUITE_HPP

#include <boost/test/unit_test.hpp>
#include <rw/proximity/CollisionStrategy.hpp>

class CollisionTestSuite : public boost::unit_test::test_suite{
public:
    CollisionTestSuite(
        const std::vector<rw::proximity::CollisionStrategyPtr>& strategies);
};

#endif // COLLISIONTESTSUITE_HPP
