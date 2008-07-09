#include "CollisionTestSuite.hpp"

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/proximity/ProximityCommon.hpp>

#include <rwlibs/drawable/Drawable.hpp>
#include <rwlibs/drawable/WorkCellGLDrawer.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/CollisionModelInfo.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyOpcode.hpp>

#include <rw/use_robwork_namespace.hpp>

#include <string>
#include <list>

using namespace boost::unit_test;

using namespace robwork;
using namespace rw::kinematics;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::drawable;

void testCDStrategyOpcode()
{

    FixedFrame* o1    = new FixedFrame("Object1", Transform3D<>::identity());
    FixedFrame* o2    = new FixedFrame("Object2", Transform3D<>::identity());

    StateStructure tree;
    Frame *world = tree.getRoot();
    tree.addFrame(o1,world);
    tree.addFrame(o2,world);

    std::cout << "Test CDStrategyOpcode" << std::endl;

    CollisionModelInfo info("#Cylinder 0.12 0.2 8");
    std::vector<CollisionModelInfo> infos(1,info);
    Accessor::collisionModelInfo().set(*o1, infos);
    Accessor::collisionModelInfo().set(*o2, infos);

    Transform3D<> wTo1(Transform3D<>::identity());
    Transform3D<> wTo2(Transform3D<>::identity());
    ProximityStrategyOpcode strategy;

    bool result;

    result = strategy.inCollision(o1, wTo1, o2, wTo2);

    BOOST_CHECK(result);

    wTo1 = Transform3D<>(Vector3D<>(0.1, 0.0, 0.0), Rotation3D<>::identity());

    result = strategy.inCollision(o1, wTo1, o2, wTo2);
    BOOST_CHECK(result);

    wTo1 = Transform3D<>(Vector3D<>(10.0, 0.0, 0.0), Rotation3D<>::identity());

    result = strategy.inCollision(o1, wTo1, o2, wTo2);
    BOOST_CHECK(!result);

    result = strategy.inCollision(o1, wTo1, o1, wTo1);
    BOOST_CHECK(result);
}

void testCDStrategy()
{
    std::cout << "Test CDStrategy\n";

    MovableFrame* cube1 = new MovableFrame("cube1");
    MovableFrame* cube2 = new MovableFrame("cube2");

    StateStructure* tree = new StateStructure();
    Frame *world = tree->getRoot();
    tree->addFrame(cube1, world);
    tree->addFrame(cube2, world);
    WorkCell workcell(tree, "testCDStrategy");

    State state = workcell.getDefaultState();
    cube1->setTransform(Transform3D<>::identity(), state);
    cube2->setTransform(Transform3D<>::identity(), state);

    CollisionModelInfo info("#Box 0.2 0.2 0.2");
    std::vector<CollisionModelInfo> infos(1,info);
    Accessor::collisionModelInfo().set(*cube1, infos);
    Accessor::collisionModelInfo().set(*cube2, infos);

    bool result;

    // The collision checking setup. No pairs are excluded.
    const ProximityPairList exclude_pairs;
    const CollisionSetup setup(exclude_pairs);

    CollisionDetector detector(
        &workcell,
        new ProximityStrategyOpcode(),
        setup);

    result = detector.inCollision(state);
    BOOST_CHECK(result);

    cube1->setTransform(
        Transform3D<>(
            Vector3D<>(10.0, 0.0, 0.0),
            Rotation3D<>::identity()),
        state);

    result = detector.inCollision(state);
    BOOST_CHECK(!result);
}

void testCDStrategyFULL()
{
    std::cout << "Test CDStrategyFull\n";

    std::auto_ptr<WorkCell> workcell =
        WorkCellLoader::load("testfiles/MultiRobotDemo/Scene.wu");

    std::auto_ptr<CollisionDetector> detector(
        new CollisionDetector(workcell.get(), new ProximityStrategyOpcode()));

    State state = workcell->getDefaultState();

    // MultiRobotDemo is in collision at its base setting ( the welding gun of
    // the first robot touches the environment

    bool result = detector->inCollision(state);

    BOOST_CHECK(true == result);

    // we just move it a bit (to a collision free setting)
    std::vector<Device*> devices = workcell->getDevices();

    std::cout
        << "Tool Pos = "
        << devices[0]->baseTend(state)
        << "\n";

    Q q = devices[0]->getQ(state);
    q[0] = 1.0;
    devices[0]->setQ(q, state);
    std::cout
        << "Tool Pos = "
        << devices[0]->baseTend(state)
        << "\n";

    std::cout << devices[0]->getQ(state) << std::endl;

    result = detector->inCollision(state);
    BOOST_CHECK(false == result);

    // we move it back to initial configuration
    q[0] = 0.0;
    devices[0]->setQ(q, state);
    result = detector->inCollision(state);
    BOOST_CHECK(true == result);
}

void testCollisionDetector()
{
    std::cout<<"Test CollisionDetector"<<std::endl;
    std::auto_ptr<WorkCell> workcell =
        WorkCellLoader::load("testfiles/MultiRobotDemo/Scene.wu");

    std::auto_ptr<CollisionDetector> detector(
        new CollisionDetector(workcell.get(), new ProximityStrategyOpcode()));

    detector->setFirstContact(false);

    State state = workcell->getDefaultState();

    // MultiRobotDemo is in collision at its base setting ( the welding gun of
    // the first robot touches the environment
    bool result = detector->inCollision(state);
    BOOST_CHECK(true == result);

    // we just move it a bit
    std::vector<Device*> devices = workcell->getDevices();
    Q q = devices[0]->getQ(state);
    q[0] = 1.0;
    devices[0]->setQ(q, state);

    result = detector->inCollision(state);
    BOOST_CHECK(false == result);

    // We move it back to initial configuration
    q[0] = 0.0;
    devices[0]->setQ(q, state);
    FramePairList resultList;
    result = detector->inCollision(state, &resultList);
    BOOST_CHECK(true == result);

    BOOST_CHECK(resultList.size() != 0);
    BOOST_REQUIRE(resultList.size() != 0);

    WorkCellGLDrawer drawer;

    bool inside = false;
    typedef FramePairList::const_iterator I;
    for (I itRes = resultList.begin(); itRes != resultList.end(); itRes++){
        std::cout
            << "Frame1: "
            << itRes->first->getName()
            << " Frame2: "
            << itRes->second->getName()
            << std::endl;

        std::vector<Drawable*> frameDrawables;
        frameDrawables = drawer.getDrawablesForFrame(itRes->first);
        BOOST_CHECK(frameDrawables.size() != 0);
        frameDrawables = drawer.getDrawablesForFrame(itRes->second);
        BOOST_CHECK(frameDrawables.size() != 0);

        inside = true;
    }
    BOOST_CHECK(inside == true);
}

void CollisionMessage(){
    BOOST_MESSAGE("CollisionTestSuite");
}

CollisionTestSuite::CollisionTestSuite() :
    boost::unit_test::test_suite("CollisionTestSuite")
{
    add( BOOST_TEST_CASE( &CollisionMessage ));
    add( BOOST_TEST_CASE( &testCDStrategyOpcode ) );
    add( BOOST_TEST_CASE( &testCDStrategy ) );
    add( BOOST_TEST_CASE( &testCDStrategyFULL ) );
    add( BOOST_TEST_CASE( &testCollisionDetector ) );
}
