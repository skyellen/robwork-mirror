#include "../TestSuiteConfig.h"
#include "PathPlanningTestSuite.hpp"

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyOpcode.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>

using namespace boost::unit_test;
using namespace robwork;

void testPathPlanning()
{
    BOOST_MESSAGE("PathPlanningTestSuite");
    WorkCellPtr workcell = WorkCellLoader::load(
        testFilePath + "simple/workcell.wu");

    Device* device = workcell->findDevice("Device");
    const State& state = workcell->getDefaultState();
    const PlannerConstraint constraint = PlannerConstraint::make(
        ProximityStrategyOpcode::make(), workcell, device, state);

    QToQPlannerPtr line = QToQPlanner::make(constraint);
	QToQPlannerPtr rrt = RRTPlanner::makeQToQPlanner(constraint, device);
	QToQPlannerPtr sbl = SBLPlanner::makeQToQPlanner(
        SBLSetup::make(constraint, device));

    const Q from = device->getQ(state);
    bool res;

    // Plan a couple of straight-line paths.
    {
        Q q(7);
        {
            int i = 0;
            q[i++] = 0.854336;
            q[i++] = 1.28309;
            q[i++] = -1.58383;
            q[i++] = -0.822832;
            q[i++] = -0.362664;
            q[i++] = -0.989248;
            q[i++] = -0.376991;
        }

        const Q linearToGood = q;
        Q linearToBad = q;
        linearToBad[0] = 2.399;

        QPath path;
        res = line->query(from, linearToGood, path, 2);
        BOOST_CHECK(res);
        BOOST_CHECK(!PlannerUtil::inCollision(path, constraint));

        res = line->query(from, linearToBad, path, 2);
        BOOST_CHECK(!res);
    }

    // Plan some paths to random configurations with RRT and SBL.
    {
        QSamplerPtr cfreeQ = QSampler::makeConstrained(
            QSampler::makeUniform(device),
            constraint.getQConstraintPtr(),
            1000);

        std::cout << "- RRT and SBL paths: ";
        for (int i = 0; i < 10; i++) {
            const Q to = cfreeQ->sample();

            QPath p1;
            res = rrt->query(from, to, p1, 4);
            BOOST_CHECK(res);
            BOOST_CHECK(!PlannerUtil::inCollision(p1, constraint));

            QPath p2;
            res = sbl->query(from, to, p2, 4);
            BOOST_CHECK(res);
            BOOST_CHECK(!PlannerUtil::inCollision(p1, constraint));
            std::cout << i << " ";
        }
        std::cout << "\n";
    }
}

PathPlanningTestSuite::PathPlanningTestSuite() :
    boost::unit_test::test_suite("PathPlanningTestSuite")
{
    add(BOOST_TEST_CASE( &testPathPlanning));
}
