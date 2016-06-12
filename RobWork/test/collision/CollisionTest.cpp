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

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>

#if RW_HAVE_PQP == 1
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#endif
#if RW_HAVE_YAOBI == 1
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#endif

#include <rw/proximity/rwstrategy/ProximityStrategyRW.hpp>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace boost::unit_test;
using namespace rwlibs::proximitystrategies;

namespace
{
    std::vector<CollisionStrategy::Ptr> allCollisionStrategies()
    {
        std::vector<CollisionStrategy::Ptr> result;
        result.push_back( ownedPtr( new ProximityStrategyRW() ) );
#if RW_HAVE_PQP == 1
        result.push_back(ProximityStrategyPQP::make());
#endif
#if RW_HAVE_YAOBI == 1
        result.push_back(ProximityStrategyYaobi::make());
#endif
        return result;
    }
}


std::vector<PlannerConstraint> getConstraints(
    const std::vector<CollisionStrategy::Ptr>& strategies,
    WorkCell::Ptr workcell,
    Device::Ptr device,
    const State& state)
{
    std::vector<PlannerConstraint> result;
    BOOST_FOREACH(const CollisionStrategy::Ptr& strategy, strategies) {
        strategy->clear(); // Sigh.
        result.push_back(
            PlannerConstraint::make(
                strategy, workcell, device, state));
    }
    return result;
}
/*
void testCollisionStrategies(const std::vector<CollisionStrategyPtr>& strategies)
{

    RW_ASSERT(!strategies.empty());

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

    const std::vector<PlannerConstraint> constraints =
        getConstraints(strategies, workcell, device, state);

    for (size_t i = 0; i < sizeof(qs) / sizeof(*qs); i++) {
        Q q(7);
        for (int j = 0; j < 7; j++) q[j] = qs[i][j];

        const bool b0 = constraints[0].getQConstraint().inCollision(q);
        for (size_t j = 1; j < constraints.size(); j++) {
            const bool bJ = constraints[j].getQConstraint().inCollision(q);

            BOOST_CHECK(b0 == bJ);
        }
    }

    // To avoid a compiler warning in case we have only one strategy.
    BOOST_CHECK(true);
}
*/
void testStrategy0(const CollisionStrategy::Ptr& strategy)
{
	BOOST_TEST_MESSAGE("- Test Strategy0");

    strategy->clear();

    const Transform3D<> id = Transform3D<>::identity();

    FixedFrame* o1 = new FixedFrame("Object1", id);
    FixedFrame* o2 = new FixedFrame("Object2", id);

    StateStructure tree;
    Frame* world = tree.getRoot();
    tree.addFrame(o1, world);
    tree.addFrame(o2, world);

    Geometry::Ptr geom = GeometryFactory::load( "#Cylinder 0.12 0.2 8" );
    strategy->addModel(o1,geom);
    strategy->addModel(o2,geom);

    const Transform3D<> a(Vector3D<>(0.1, 0.0, 0.0));
    BOOST_CHECK(strategy->inCollision(o1, a, o2, id));

    const Transform3D<> b(Vector3D<>(10.0, 0.0, 0.0));
    BOOST_CHECK(!strategy->inCollision(o1, b, o2, id));

}

void testStrategy1(const CollisionStrategy::Ptr& strategy, int i)
{
	BOOST_TEST_MESSAGE("- Test Strategy1");

    strategy->clear();

    MovableFrame* cube1 = new MovableFrame("cube1");
    MovableFrame* cube2 = new MovableFrame("cube2");

    StateStructure tree;
    Frame *world = tree.getRoot();
    tree.addFrame(cube1, world);
    tree.addFrame(cube2, world);
    WorkCell workcell(&tree, "testStrategy");

    const Transform3D<> id = Transform3D<>::identity();
    State state = workcell.getDefaultState();
    cube1->setTransform(id, state);
    cube2->setTransform(id, state);

    Geometry::Ptr geom = GeometryFactory::load( "#Box 0.2 0.2 0.2" );
    strategy->addModel(cube1,geom);
    strategy->addModel(cube2,geom);

    bool result;

    CollisionDetector detector(&workcell, strategy);
    detector.addGeometry( cube1,geom );
    detector.addGeometry( cube2,geom );
    detector.addRule( ProximitySetupRule::makeInclude(cube1->getName(), cube2->getName()) );

    result = detector.inCollision(state);
    BOOST_CHECK_MESSAGE(result, "Collision result is not correct! Strategy, " << i << " is faulty!");

    cube1->setTransform(Transform3D<>(Vector3D<>(10.0, 0.0, 0.0)), state);

    result = detector.inCollision(state);
    BOOST_CHECK(!result);

}

void testCollisionDetector(const CollisionStrategy::Ptr& strategy)
{

}


BOOST_AUTO_TEST_CASE( mainCollisionTest )
{

	std::vector<CollisionStrategy::Ptr> strategies = allCollisionStrategies();

	int idx = 0;

    BOOST_FOREACH(const CollisionStrategy::Ptr& strategy, strategies) {

        testStrategy0(strategy);

        testStrategy1(strategy, idx);

        //testCollisionDetector(strategy);
        idx++;
    }


    if (strategies.empty()) {
		BOOST_TEST_MESSAGE("No collision strategies available!\n");
    } else {
        //testCollisionStrategies( strategies);
    }
}
