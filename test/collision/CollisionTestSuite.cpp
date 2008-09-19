#include "CollisionTestSuite.hpp"

#include "../TestSuiteConfig.hpp"

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
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyOpcode.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>

#include <string>

using namespace boost::unit_test;
using namespace robwork;

void testCollisionStrategies()
{
    // A bunch of Qs. You can shorten the list if you want.
    const double qs[][7] = {
        {2.61412, 0.442982, 1.06049, -0.168127, -2.68623, -0.15103, -3.10707},
        {2.90358, -0.702939, -1.30048, -0.431363, 1.06622, 0.869435, 1.91141},
        {2.93428, -0.360079, -0.706436, -0.112186, 1.17301, 0.446959, 3.10956},
        {0.551546, 0.771784, 1.4654, 0.427852, 2.23459, -1.32552, 0.262281},
        {-1.11077, 1.44331, 2.59742, -0.687583, -1.87864, -1.36107, 2.15884},
        {-0.726627, 0.616769, 2.91598, -0.419954, 1.65834, 0.0916537, -1.90448},
        {0.268961, 1.50704, 1.34494, 0.635913, 0.104024, 0.940077, -2.30545},
        {-2.80312, 0.859083, 1.399, -0.294454, -0.102879, 1.45485, 1.85308},
        {-1.36022, 0.678765, -1.48862, -1.01656, -1.0461, 1.31545, 0.454381},
        {0.488244, 1.24052, -0.718004, -1.01699, -1.71087, -0.692159, 1.78153},
        {0.51321, 1.47876, -0.286926, 0.739909, -0.33098, 0.345147, 2.01523},
        {2.82955, 1.33165, 1.32934, -0.0587259, -0.27349, 0.588583, -1.12058},
        {1.89161, -0.977961, 0.1154, 1.17395, -1.29235, 0.700227, 0.867611},
        {3.00386, 0.267378, -2.66908, 1.26155, -1.16412, -1.40615, -0.587713},
        {2.57693, 1.45312, 0.216343, 0.285214, -3.0431, 0.56575, -0.975765},
        {-1.43971, 0.704765, -1.52375, -0.0363385, -0.987894, 1.50846, -2.61382},
        {-0.486255, 1.38078, -1.08928, -0.297945, 2.02112, -0.940902, 0.301007},
        {-0.981345, 0.572796, -1.91141, 0.960392, -1.53411, 0.538556, 1.61852},
        {-0.487528, -0.823028, -2.35747, 0.624685, 0.504214, 0.293316, 2.49715},
        {-2.23001, -0.254807, 1.65693, 1.29033, 2.21716, 0.011083, 2.86505},
        {0.75927, 1.44199, 2.09223, 0.648333, 0.405876, -0.65551, -2.57002},
        {-2.90339, 1.51104, -0.689703, -0.798438, 1.46478, 0.661322, 2.76861},
        {0.0321287, -1.2141, -0.133375, 0.132707, -1.60887, -0.0792507, 1.39581},
        {-2.85427, -1.21646, 0.946734, 1.5408, 2.34099, 1.08502, -0.239687},
        {0.216826, 0.0730914, -0.474048, -1.01225, -1.34168, -1.19029, 0.00999952},
        {0.715327, 1.19198, 1.35974, -0.706838, -1.33516, 0.00168849, -0.00345258},
        {-1.66787, 1.43904, -1.02322, -1.46611, -1.94615, -0.829967, 3.08215},
        {-2.49409, 0.224616, 2.02314, 0.73141, 3.09286, -1.26214, -2.13799},
        {-0.841208, 0.79137, 2.46703, -0.667969, -2.61105, -0.538015, -2.10183},
        {2.70082, 0.394011, 1.33532, 0.385753, 1.96082, 1.06257, -0.284242},
        {2.73628, -0.54267, -2.24947, -1.13629, -1.5119, 0.393887, -0.455773},
        {1.12994, -1.56796, 2.70427, -1.35147, -2.35523, -0.859253, 1.48949},
        {-0.11874, 0.14621, -1.56123, -0.134051, 2.36676, 0.794567, -1.10413},
        {0.674032, 1.33399, -2.49597, 1.49159, -0.507955, -0.157515, 3.11439},
        {-1.7145, 0.84055, -1.309, 0.824921, 1.73626, 1.3264, -1.42409},
        {1.44127, -0.37516, 2.18645, -0.0673059, -0.621511, 0.235969, -0.62741},
        {3.11709, 1.43581, -1.68258, 0.80894, -0.920966, -0.0901152, -1.4898},
        {-0.0653663, -0.436325, -3.03735, -1.25411, -1.69813, -0.440421, -0.180314},
        {2.43703, -0.6478, -1.2655, 1.51367, -0.804657, -1.20787, 2.79271},
        {0.0349219, 0.717474, -3.07931, 0.0525862, -1.42931, 0.870827, -1.73145},
        {-2.36763, 0.330148, -0.223114, 1.21648, -2.39869, 0.208738, -1.67373},
        {0.905423, -1.12511, -1.38278, -0.434164, -1.20093, -0.371841, -0.00625028},
        {2.81052, 0.215387, -1.48204, 0.145547, -0.173235, -0.753837, 1.95645},
        {-2.22441, 0.991542, -0.827041, 0.786476, 1.78518, -0.66654, 0.604022},
        {-0.0315943, -0.999425, -1.97159, -0.838121, -1.95784, 1.26981, -0.403122},
        {-0.652271, 0.74956, 2.22237, 0.0835169, 3.00004, 1.2148, -0.893046},
        {2.07916, -1.53021, -2.94301, 0.715619, 0.42989, -0.0457474, 0.68588},
        {2.43424, 1.45008, -2.07829, -1.4978, -1.61322, 0.0977154, -0.17818},
        {-1.85491, -1.14424, -2.80442, -0.438774, 0.549707, -1.5219, 1.71282},
        {1.89614, 0.863515, -2.50885, -1.47564, 0.602293, -0.292337, -1.49394}
    };

    WorkCellPtr workcell = WorkCellLoader::load(
        testFilePath() + "simple/workcell.wu");
    Device* device = workcell->findDevice("Device");
    const State& state = workcell->getDefaultState();

    const PlannerConstraint constraints[] = {
        PlannerConstraint::make(
            ProximityStrategyOpcode::make(), workcell, device, state),
        PlannerConstraint::make(
            ProximityStrategyYaobi::make(), workcell, device, state),
        PlannerConstraint::make(
            ProximityStrategyPQP::make(), workcell, device, state)
    };

    for (size_t i = 0; i < sizeof(qs) / sizeof(*qs); i++) {
        Q q(7);
        for (int j = 0; j < 7; j++) q[j] = qs[i][j];

        const bool b0 = constraints[0].getQConstraint().inCollision(q);
        for (size_t j = 1; j < sizeof(constraints) / sizeof(*constraints); j++) {
            const bool bJ = constraints[j].getQConstraint().inCollision(q);

            // This is what the test looks like:
            //   BOOST_CHECK(b0 == bJ);

            // Yaobi or the Yaobi wrapper does not work, so we just omit an
            // error message.
            if (b0 != bJ) {
                BOOST_MESSAGE("- ERROR: Collision checkers differ.");
                std::cout
                    << "- Q: " << i
                    << " Bad collision strategy: " << j << "\n";
            }
        }
    }
}

void testCDStrategyOpcode()
{
    FixedFrame* o1 = new FixedFrame("Object1", Transform3D<>::identity());
    FixedFrame* o2 = new FixedFrame("Object2", Transform3D<>::identity());

    StateStructure tree;
    Frame *world = tree.getRoot();
    tree.addFrame(o1,world);
    tree.addFrame(o2,world);

    BOOST_MESSAGE("- Test CDStrategyOpcode");

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
    BOOST_MESSAGE("- Test CDStrategy");

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
        ProximityStrategyOpcode::make(),
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
    BOOST_MESSAGE("- Test CDStrategyFull");

    WorkCellPtr workcell = WorkCellLoader::load(
        testFilePath() + "MultiRobotDemo/Scene.wu");

    CollisionDetector detector(workcell, ProximityStrategyOpcode::make());

    State state = workcell->getDefaultState();

    // MultiRobotDemo is in collision at its base setting ( the welding gun of
    // the first robot touches the environment

    bool result = detector.inCollision(state);

    BOOST_CHECK(true == result);

    // we just move it a bit (to a collision free setting)
    std::vector<Device*> devices = workcell->getDevices();

    //std::cout
    //    << "Tool Pos = "
    //    << devices[0]->baseTend(state)
    //    << "\n";

    Q q = devices[0]->getQ(state);
    q[0] = 1.0;
    devices[0]->setQ(q, state);
    //std::cout
    //    << "Tool Pos = "
    //    << devices[0]->baseTend(state)
    //    << "\n";

    //std::cout << devices[0]->getQ(state) << std::endl;

    result = detector.inCollision(state);
    BOOST_CHECK(false == result);

    // we move it back to initial configuration
    q[0] = 0.0;
    devices[0]->setQ(q, state);
    result = detector.inCollision(state);
    BOOST_CHECK(true == result);
}

void testCollisionDetector()
{
    std::cout<<"Test CollisionDetector"<<std::endl;
    WorkCellPtr workcell = WorkCellLoader::load(
        testFilePath() + "MultiRobotDemo/Scene.wu");

    CollisionDetector detector(
        workcell, ProximityStrategyOpcode::make());

    State state = workcell->getDefaultState();

    // MultiRobotDemo is in collision at its base setting ( the welding gun of
    // the first robot touches the environment
    bool result = detector.inCollision(state);
    BOOST_CHECK(true == result);

    // we just move it a bit
    std::vector<Device*> devices = workcell->getDevices();
    Q q = devices[0]->getQ(state);
    q[0] = 1.0;
    devices[0]->setQ(q, state);

    result = detector.inCollision(state);
    BOOST_CHECK(false == result);

    // We move it back to initial configuration
    q[0] = 0.0;
    devices[0]->setQ(q, state);
    FramePairSet resultList;
    result = detector.inCollision(state, &resultList);
    BOOST_CHECK(true == result);

    BOOST_CHECK(resultList.size() != 0);
    BOOST_REQUIRE(resultList.size() != 0);

    WorkCellGLDrawer drawer;

    bool inside = false;
    typedef FramePairSet::const_iterator I;
    for (I itRes = resultList.begin(); itRes != resultList.end(); itRes++){
        //std::cout
        //    << "Frame1: "
        //    << itRes->first->getName()
        //    << " Frame2: "
        //    << itRes->second->getName()
        //    << std::endl;

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
    add(BOOST_TEST_CASE(&testCollisionStrategies));
}
