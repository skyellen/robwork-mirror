#include "KinematicsTestSuite.hpp"

#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/Tree.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>

#include <boost/test/unit_test.hpp>

using namespace boost::unit_test;

using namespace rw::kinematics;
using namespace rw::math;

void singleChainTest()
{

    FixedFrame* world = new FixedFrame(NULL,"world", Transform3D<>::Identity());
    FixedFrame* l1 = new FixedFrame(world,"l1", Transform3D<>(Vector3D<>(1,2,3), RPY<>(0,0,0)));
    FixedFrame* l2 = new FixedFrame(l1, "l2", Transform3D<>(Vector3D<>(2,3,4), RPY<>(0,0,0)));
    FixedFrame* l3 = new FixedFrame(l2, "l3", Transform3D<>(Vector3D<>(3,4,5), RPY<>(0,0,0)));

    boost::shared_ptr<Tree> tree(new Tree());
    tree->addFrame(world);
    tree->addFrame(l1);
    tree->addFrame(l2);
    tree->addFrame(l3);

    State state(tree);
    Transform3D<> transform = Kinematics::FrameTframe(world, l3, state);

    BOOST_REQUIRE(transform.P()(0) == 6.0);
    BOOST_REQUIRE(transform.P()(1) == 9.0);
    BOOST_REQUIRE(transform.P()(2) == 12.0);
}

void multipleChainTest(){

    FixedFrame* world = new FixedFrame(NULL, "world", Transform3D<>::Identity());
    FixedFrame* l1 = new FixedFrame(world, "l1", Transform3D<>(Vector3D<>(1,2,3), RPY<>(0,0,0)));
    FixedFrame* l2 = new FixedFrame(world, "l2", Transform3D<>(Vector3D<>(2,3,4), RPY<>(0,0,0)));

    boost::shared_ptr<Tree> tree(new Tree());
    tree->addFrame(world);
    tree->addFrame(l1);
    tree->addFrame(l2);

    State state(tree);
    Transform3D<> transform = Kinematics::FrameTframe(world, l1, state);
    BOOST_REQUIRE(transform.P()(0) == 1.0);
    BOOST_REQUIRE(transform.P()(1) == 2.0);
    BOOST_REQUIRE(transform.P()(2) == 3.0);

    transform = Kinematics::FrameTframe(world, l2, state);

    BOOST_REQUIRE(transform.P()(0) == 2.0);
    BOOST_REQUIRE(transform.P()(1) == 3.0);
    BOOST_REQUIRE(transform.P()(2) == 4.0);
}

KinematicsTestSuite::KinematicsTestSuite()
{
    add( BOOST_TEST_CASE( &singleChainTest) );
    add( BOOST_TEST_CASE( &multipleChainTest) );
}
