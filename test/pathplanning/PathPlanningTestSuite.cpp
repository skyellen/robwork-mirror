#include "../TestSuiteConfig.h"
#include "PathPlanningTestSuite.hpp"

#include <rw/pathplanning/PathPlanner.hpp>
#include <rw/pathplanning/StraightLinePathPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>

#include <rwlibs/pathplanners/lazyprm/LazyPRMPathPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPathPlanner.hpp>

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
    std::auto_ptr<WorkCell> workcell =
        WorkCellLoader::load(testFilePath+"MultiRobotDemo/Scene.wu");

    Device* device = workcell->getDevices()[0];
    //    CDStrategyOpcode opcode;
    RRTPathPlanner rrt(
        workcell.get(),
        device,
        new CollisionDetector(
            workcell.get(),
            new ProximityStrategyOpcode()),
        workcell->getDefaultState(),
        0.01);

    LazyPRMPathPlanner lazy(
        workcell.get(),
        device,
        new CollisionDetector(
            workcell.get(), new ProximityStrategyOpcode()), 0.01);

    lazy.initialize(device);

    StraightLinePathPlanner line(
        device,
        workcell->getDefaultState(),
        new CollisionDetector(
            workcell.get(), new ProximityStrategyOpcode()), 0.01);

    Q qInit(9);
    Q qGoal(9);
    for(int i = 0; i<9; i++){
        qInit[i] = qGoal[i] = 0.0;
    }
    qInit[0] = 1.99;
    qGoal[0] = 0.92;

    bool res;
    Path path;

    path.clear();
    res = line.query(qInit, qGoal, path, 60);
    BOOST_CHECK(res);

    path.clear();
    res = rrt.query(qInit, qGoal, path, 60);
    BOOST_CHECK(res);

    Path::iterator it;
    for(it = path.begin(); it != path.end(); ++it){
        std::cout << (*it) << std::endl;
    }

    path.clear();
    res = lazy.query(qInit, qGoal, path, 60);
    BOOST_CHECK(res);
}

PathPlanningTestSuite::PathPlanningTestSuite() :
    boost::unit_test::test_suite("PathPlanningTestSuite")
{

    add( BOOST_TEST_CASE( &testPathPlanning) );
}
