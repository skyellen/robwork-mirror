#include "../TestSuiteConfig.h"
#include "PathPlanningTestSuite.hpp"

#include <rw/pathplanning/QToQPlanner.hpp>
// #include <rw/pathplanning/StraightLineQToQPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>

// #include <rwlibs/pathplanners/lazyprm/LazyPRMQToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyOpcode.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/models/SerialDevice.hpp>

#include <rw/math/EAA.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Rotation3D.hpp>

#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/use_robwork_namespace.hpp>

using namespace boost::unit_test;

using namespace robwork;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

void testPathPlanning()
{
    BOOST_MESSAGE("PathPlanningTestSuite");
    WorkCellPtr workcell =
        WorkCellLoader::load(testFilePath + "MultiRobotDemo/Scene.wu");

    Device* device = workcell->findDevice("PA10_1");

    ProximityStrategyOpcode strategy;

    CollisionDetector detector(workcell, &strategy);

    QConstraintPtr constraint =
        QConstraint::make(
            &detector,
            device,
            workcell->getDefaultState());

    QEdgeConstraintPtr edge =
        QEdgeConstraint::make(
			constraint,
            Metric<>::makeEuclidean(),
            0.01);

    QSamplerPtr sampler = QSampler::makeUniform(*device);

    QToQPlannerPtr line = QToQPlanner::make(constraint, edge);
    RRTQToQPlanner rrt(constraint, edge, sampler);

    Q qInit(9);
    Q qGoal(9);
    for(int i = 0; i < 9; i++) {
        qInit[i] = qGoal[i] = 0.0;
    }
    qInit[0] = 1.99;
    qGoal[0] = 0.92;

    bool res;
    Path path;

    res = line->query(qInit, qGoal, path, 60);
    BOOST_CHECK(res);

    res = rrt.query(qInit, qGoal, path, 60);
    BOOST_CHECK(res);
}

PathPlanningTestSuite::PathPlanningTestSuite() :
    boost::unit_test::test_suite("PathPlanningTestSuite")
{

    add( BOOST_TEST_CASE( &testPathPlanning) );
}
