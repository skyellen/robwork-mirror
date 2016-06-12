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
#include <rw/geometry/DistanceUtil.hpp>

#if RW_HAVE_PQP == 1
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#endif

#include <rw/proximity/rwstrategy/ProximityStrategyRW.hpp>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace boost::unit_test;
using namespace rwlibs::proximitystrategies;

namespace
{
    std::vector<DistanceStrategy::Ptr> allDistanceStrategies()
    {
        std::vector<DistanceStrategy::Ptr> result;
        //result.push_back( ownedPtr( new ProximityStrategyRW() ) );
#if RW_HAVE_PQP == 1
        result.push_back(ownedPtr( new ProximityStrategyPQP() ) );
#endif
        return result;
    }
}

void testStrategy0(const DistanceStrategy::Ptr& strategy)
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

    Geometry::Ptr geom1 = Geometry::makeCylinder(0.0001f,10.f);
    Geometry::Ptr geom2 = Geometry::makeCylinder(0.0001f,10.f);
    strategy->addModel(o1, geom1);
    strategy->addModel(o2, geom2);

    // check if distance between z-rays correspond to distance between cylinders
    for(int i=0; i<10000; i++){
        Vector3D<> v1( Math::ran(-3,3), Math::ran(-3,3), Math::ran(-3,3));
        Vector3D<> v2( Math::ran(-3,3), Math::ran(-3,3), Math::ran(-3,3));

        Rotation3D<> rot1 = Math::ranRotation3D<double>();
        Rotation3D<> rot2 = Math::ranRotation3D<double>();

        Transform3D<> t1(v1,rot1);
        Transform3D<> t2(v2,rot2);
        //std::cout << t1 << std::endl;
        //std::cout << t2 << std::endl;

        double dist1 = strategy->distance(o1,t1,o2,t2).distance;

        Vector3D<> p1 = t1.P()-t1.R()*Vector3D<>::z()*5;
        Vector3D<> p2 = t1.P()+t1.R()*Vector3D<>::z()*5;
        Vector3D<> q1 = t2.P()-t2.R()*Vector3D<>::z()*5;
        Vector3D<> q2 = t2.P()+t2.R()*Vector3D<>::z()*5;

        double dist2 = DistanceUtil::distanceLineLine(p1,p2,q1,q2);

        // distance must be within 0.03
        //std::cout << fabs(dist1 - dist2) << "\t" << dist1 << " - " << dist2 << std::endl;
        BOOST_CHECK_SMALL( fabs(dist1 - dist2), 0.003);
    }

}

BOOST_AUTO_TEST_CASE( mainDistanceTest )
{

	std::vector<DistanceStrategy::Ptr> strategies = allDistanceStrategies();

	int idx = 0;

    BOOST_FOREACH(const DistanceStrategy::Ptr& strategy, strategies) {

        testStrategy0(strategy);

        //testStrategy1(strategy, idx);

        idx++;
    }


    if (strategies.empty()) {
		BOOST_TEST_MESSAGE("No collision strategies available!\n");
    } else {
        //testCollisionStrategies( strategies);
    }
}
