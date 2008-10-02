#include "../TestSuiteConfig.hpp"
#include "PathPlanningTestSuite.hpp"

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/pathplanners/arw/ARWPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>

#include <rwlibs/pathplanners/prm/PartialIndexTable.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyOpcode.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/math/MetricFactory.hpp>
#include <boost/foreach.hpp>

#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>

using namespace boost::unit_test;
using namespace robwork;
using namespace rwlibs::pathplanners::prm;

void testPartialIndexTable()
{
    Q lower = Q::zero(3);
    Q upper(3); upper(0) = upper(1) = upper(2) = 1;
    const double radius = 0.1;
    Q weights(3); weights(0) = weights(1) = weights(2) = radius;
    const Device::QBox bounds(lower, upper);

    typedef PartialIndexTable<Q> Table;
    Table table(
        bounds,
        weights,
        radius,
        3);

    QSamplerPtr anyQ = QSampler::makeUniform(bounds);

    for (int i = 0; i < 20; i++) {
        Q q = anyQ->sample();
        table.addNode(q, q);
    }

    // Now search for neighbors.
    const std::list<Q> neighbors = table.searchNeighbors(lower);

    QMetricPtr metric = MetricFactory::makeInfinity<Q>();

    bool ok = true;
    BOOST_FOREACH(const Q& q, neighbors) {
        const double dist = metric->distance(lower, q);

        const bool bad = dist > 2 * radius;
        if (bad) {
            std::cout
                << "PartialIndexTable::searchNeighbors(): Distance "
                << dist
                << " to neighbor too great." << std::endl;
        }

        ok = ok && !bad;
    }

    BOOST_CHECK(ok);
}

void testPathPlanning()
{
    BOOST_MESSAGE("PathPlanningTestSuite");
    WorkCellPtr workcell = WorkCellLoader::load(
        testFilePath() + "simple/workcell.wu");

    Device* device = workcell->findDevice("Device");
    const State& state = workcell->getDefaultState();
    const PlannerConstraint constraint = PlannerConstraint::make(
        ProximityStrategyOpcode::make(), workcell, device, state);

    QToQPlannerPtr line = QToQPlanner::make(constraint);

    const QToQPlannerPtr plannersArray[] = {
        RRTPlanner::makeQToQPlanner(constraint, device),
        ARWPlanner::makeQToQPlanner(constraint, device),
        SBLPlanner::makeQToQPlanner(SBLSetup::make(constraint, device))
    };
    const std::vector<QToQPlannerPtr> planners(
        plannersArray, plannersArray + sizeof(plannersArray) / sizeof(*plannersArray));

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

        std::cout << "- RRT, ARW, and SBL paths: ";
        for (int i = 0; i < 10; i++) {
            const Q to = cfreeQ->sample();
            BOOST_FOREACH(const QToQPlannerPtr& planner, planners) {
                QPath path;
                res = planner->query(from, to, path, 4);
                BOOST_CHECK(res);
                BOOST_CHECK(!PlannerUtil::inCollision(path, constraint));
            }
            std::cout << i << " ";
        }
        std::cout << "\n";
    }
}

PathPlanningTestSuite::PathPlanningTestSuite() :
    boost::unit_test::test_suite("PathPlanningTestSuite")
{
    add(BOOST_TEST_CASE(&testPartialIndexTable));

    add(BOOST_TEST_CASE(&testPathPlanning));
}
