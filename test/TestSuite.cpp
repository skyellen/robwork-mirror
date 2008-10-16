#include "TestSuiteConfig.hpp"

#include "math/MathTestSuite.hpp"
#include "kinematics/KinematicsTestSuite.hpp"
#include "models/ModelsTestSuite.hpp"
#include "collision/CollisionTestSuite.hpp"
#include "pathplanning/PathPlanningTestSuite.hpp"

#include "drawable/DrawableTestSuite.hpp"
#include "invkin/InvKinTestSuite.hpp"

#include "trajectory/TrajectoryTestSuite.hpp"
#include "loaders/TULTestSuite.hpp"
#include "common/CommonTestSuite.hpp"

#if RW_HAVE_PQP == 1
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#endif
#if RW_HAVE_YAOBI == 1
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#endif

#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>
using namespace robwork;

#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test_framework.hpp>

using boost::unit_test::test_suite;

namespace
{
    std::vector<CollisionStrategyPtr> allCollisionStrategies()
    {
        std::vector<CollisionStrategyPtr> result;
#if RW_HAVE_PQP == 1
        result.push_back(ProximityStrategyPQP::make());
#endif
#if RW_HAVE_YAOBI == 1
        result.push_back(ProximityStrategyYaobi::make());
#endif
        return result;
    }
}

test_suite* init_unit_test_suite(int argc, char* argv[])
{
    boost::unit_test::unit_test_log.set_threshold_level(boost::unit_test::log_messages);

    test_suite* test = BOOST_TEST_SUITE("RobWork test suite");
    test->add(new MathTestSuite);
    test->add(new KinematicsTestSuite);
    test->add(new ModelsTestSuite);
    test->add(new TULTestSuite);
    test->add(new InvKinTestSuite);
    test->add(new DrawableTestSuite);

    test->add(new CollisionTestSuite(allCollisionStrategies()));

#if RW_HAVE_YAOBI == 1
    test->add(new PathPlanningTestSuite(ProximityStrategyYaobi::make()));
#endif

    test->add(new TrajectoryTestSuite);
    test->add(new CommonTestSuite);
    return test;
}
