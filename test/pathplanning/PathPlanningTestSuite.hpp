#ifndef PATHPLANNINGTESTSUITE_HPP
#define PATHPLANNINGTESTSUITE_HPP

#include <boost/test/unit_test.hpp>
#include <rw/proximity/CollisionStrategy.hpp>

class PathPlanningTestSuite :
    public boost::unit_test::test_suite
{
public:
    PathPlanningTestSuite(
        rw::proximity::CollisionStrategyPtr strategy);
};

#endif // PATHPLANNINGTESTSUITE_HPP
