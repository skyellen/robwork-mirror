/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#include "../TestSuiteConfig.hpp"

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/pathplanners/arw/ARWPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/prm/PartialIndexTable.hpp>

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/math/MetricFactory.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>

#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>


#include <boost/test/unit_test.hpp>

#if RW_HAVE_PQP == 1
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#endif
#if RW_HAVE_YAOBI == 1
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#endif

using namespace rwlibs::proximitystrategies;
using namespace boost::unit_test;
using namespace robwork;
using namespace rwlibs::pathplanners::prm;

BOOST_AUTO_TEST_CASE( testPartialIndexTable )
{
    Q lower = Q::zero(3);
    Q upper(3); upper(0) = upper(1) = upper(2) = 1;
    const double radius = 0.1;
    Q weights(3); weights(0) = weights(1) = weights(2) = 1;
    const Device::QBox bounds(lower, upper);

    typedef PartialIndexTable<Q> Table;
    Table table(
        bounds,
        weights,
        radius,
        3);

    QSamplerPtr anyQ = QSampler::makeUniform(bounds);

    for (int i = 0; i < 1000; i++) {
        Q q = anyQ->sample();
        table.addNode(q, q);
    }

    const Q pos = lower;

    // Now search for neighbors near pos.
    std::vector<Q> neighbors;
    table.searchNeighbors(pos, neighbors);

    QMetricPtr metric = MetricFactory::makeInfinity<Q>();

    bool ok = true;
    BOOST_FOREACH(const Q& q, neighbors) {
        const double dist = metric->distance(pos, q);

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
/*
void testPathPlanning(const CollisionStrategyPtr& strategy)
{
    BOOST_MESSAGE("PathPlanningTestSuite");
    WorkCellPtr workcell = WorkCellLoader::load(
        testFilePath() + "simple/workcell.wu");

    Device* device = workcell->findDevice("Device");
    const State& state = workcell->getDefaultState();
    const PlannerConstraint constraint = PlannerConstraint::make(
        strategy, workcell, device, state);

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
        BOOST_CHECK(!PlannerUtil::inCollision(constraint, path));

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
                BOOST_CHECK(!PlannerUtil::inCollision(constraint, path));
            }
            std::cout << i << " ";
        }
        std::cout << "\n";
    }
}
*/
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

BOOST_AUTO_TEST_CASE( testPathPlanningMain )
{
/*
    std::vector<CollisionStrategyPtr> strategies = allCollisionStrategies();
	BOOST_FOREACH(CollisionStrategyPtr &sptr, strategies){
		testPathPlanning(sptr);
	}
	*/
}
