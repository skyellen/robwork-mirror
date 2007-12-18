#include "math/MathTestSuite.hpp"
#include "kinematics/KinematicsTestSuite.hpp"
#include "models/ModelsTestSuite.hpp"
#include "collision/CollisionTestSuite.hpp"
#include "drawable/DrawableTestSuite.hpp"
#include "invkin/InverseKinematicsTestSuite.hpp"
#include "pathplanning/PathPlanningTestSuite.hpp"
#include "interpolator/InterpolatorTestSuite.hpp"
#include "interpolator/InterpolatorTestSuite.hpp"
#include "loaders/TULTestSuite.hpp"
#include "common/CommonTestSuite.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test_framework.hpp>

using boost::unit_test::test_suite;

test_suite* init_unit_test_suite(int argc, char* argv[])
{
    test_suite* test = BOOST_TEST_SUITE("RobWork test suite");

    test->add(new MathTestSuite);
    test->add(new KinematicsTestSuite);
    test->add(new ModelsTestSuite);
    test->add(new TULTestSuite);
    test->add(new InvKinTestSuite);
    test->add(new DrawableTestSuite);
    test->add(new CollisionTestSuite);
    test->add(new InterpolatorTestSuite);
    test->add(new PathPlanningTestSuite);
    test->add(new CommonTestSuite);
    return test;
}
