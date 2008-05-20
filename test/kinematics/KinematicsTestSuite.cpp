#include "KinematicsTestSuite.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <boost/foreach.hpp>

using namespace boost::unit_test;

using namespace rw::kinematics;
using namespace rw::math;

void sharedPtrTest(){
    BOOST_MESSAGE("Shared PTR test");
    typedef std::vector<boost::shared_ptr<Frame> > SharedVector;
    int N=10;
    SharedVector f1,f2;
    {
        SharedVector frames(N);
        for(int i = 0;i<N; i++){
            std::ostringstream ostr;
            ostr << "L" << i;
            frames[i] = boost::shared_ptr<Frame>(new FixedFrame(ostr.str(), Transform3D<>(Vector3D<>(1,2,3), RPY<>(0,0,0))));
        }
        f1 = frames;
    }
    {
        SharedVector vf = f1;
    }
    {
        SharedVector vf = f1;
        f2 = vf;
    }
    BOOST_FOREACH(boost::shared_ptr<Frame>& frame, f1){
        std::cout << frame->getName() << " " << frame.use_count() << std::endl;        
    }
    BOOST_FOREACH(boost::shared_ptr<Frame>& frame, f2){
        std::cout << frame->getName() << " " << frame.use_count() << std::endl;        
    }
}

void StateStructureTest()
{
    sharedPtrTest();
    
    BOOST_MESSAGE("Testing StateStructure ");
    
    FixedFrame* l1 = new FixedFrame("l1", Transform3D<>(Vector3D<>(1,2,3), RPY<>(0,0,0)));
    MovableFrame* m1 = new MovableFrame("m1");
    FixedFrame* daf = new FixedFrame("daf", Transform3D<>(Vector3D<>(1,2,3), RPY<>(0,0,0)));
    
    BOOST_MESSAGE("- Creating StateStructure");
    boost::shared_ptr<StateStructure> tree( new StateStructure() );
    Frame* world = tree->getRoot();
    BOOST_MESSAGE("- Testing insertion in tree");
    std::cout << "a";
    tree->addFrame(l1,world);
    std::cout << "a";
    tree->addFrame(m1,world);
    std::cout << "a";
    tree->addDAF(daf,world);
    // Todo: check if frames are in the tree, and if parents was set correctly
    
    BOOST_MESSAGE("- Getting default state");
    State state = tree->getDefaultState();
    
    BOOST_MESSAGE("- Testing QState func");
    Transform3D<> m1_t3d( Vector3D<>(1,2,3), RPY<>(0,0,0) );
    m1->setTransform(m1_t3d, state);
    Transform3D<> m1_t3d_b; 
    m1_t3d_b= m1->getTransform(state);
    BOOST_REQUIRE_CLOSE( m1_t3d_b.P()[0], m1_t3d.P()[0] , 1e-6);
    BOOST_REQUIRE_CLOSE( m1_t3d_b.P()[1], m1_t3d.P()[1] , 1e-6);
    BOOST_REQUIRE_CLOSE( m1_t3d_b.P()[2], m1_t3d.P()[2] , 1e-6);

    BOOST_MESSAGE("- Testing daf func");
    BOOST_REQUIRE_EQUAL( world , daf->getParent(state) );
    daf->attachTo(m1, state);
    BOOST_REQUIRE_EQUAL( m1 , daf->getParent(state) );
    daf->attachTo(l1, state);
    BOOST_REQUIRE_EQUAL( l1 , daf->getParent(state) );
    
    BOOST_MESSAGE("- Testing adding and deleting of frames");
    tree->setDefaultState(state);
    // todo: test delete func, adding has allready been tested
    tree->remove(l1);
    
    // the daf should have changed its parent to world in the default state
    BOOST_MESSAGE("A");
    Frame *dafParent = daf->getParent(tree->getDefaultState());
    BOOST_REQUIRE_EQUAL(world, dafParent);
    // now add a new l1 frame. Remember the tree took ownership of the old
    // l1 frame so we are not allowed to use that again
    l1 = new FixedFrame("l1b", Transform3D<>(Vector3D<>(1,2,3), RPY<>(0,0,0)));
    BOOST_MESSAGE("A");
    tree->addFrame(l1,world);
    state = tree->upgradeState(state);
    BOOST_MESSAGE("A");
    daf->attachTo(l1, state);
    BOOST_MESSAGE("A");
    tree->setDefaultState(state);
    
    BOOST_MESSAGE("- Testing copy and upgrade of State");
    // todo: test copy of state func
    MovableFrame* m2 = new MovableFrame("m2");
    tree->addFrame(m2,world);
    State nstate = tree->getDefaultState();
    nstate.copy(state);

    BOOST_REQUIRE_EQUAL( l1 , daf->getParent(nstate) );
    m1_t3d_b= m1->getTransform(nstate);
    BOOST_REQUIRE_CLOSE( m1_t3d_b.P()[0], m1_t3d.P()[0] , 1e-6);
    BOOST_REQUIRE_CLOSE( m1_t3d_b.P()[1], m1_t3d.P()[1] , 1e-6);
    BOOST_REQUIRE_CLOSE( m1_t3d_b.P()[2], m1_t3d.P()[2] , 1e-6);
    
    // we uprade the old state and check if the newly added frame is there
    state = tree->upgradeState(state);
    Transform3D<> m2_t3d( Vector3D<>(1,2,3), RPY<>(0,0,0) );
    m2->setTransform(m2_t3d, state);
    Transform3D<> m2_t3d_b;
    m2_t3d_b= m2->getTransform(state);
    BOOST_REQUIRE_CLOSE( m2_t3d_b.P()[0], m2_t3d.P()[0] , 1e-6);
    BOOST_REQUIRE_CLOSE( m2_t3d_b.P()[1], m2_t3d.P()[1] , 1e-6);
    BOOST_REQUIRE_CLOSE( m2_t3d_b.P()[2], m2_t3d.P()[2] , 1e-6);
    
    // also check if old state values are not deleted    
    BOOST_REQUIRE_EQUAL( l1 , daf->getParent(state) );
    m1_t3d_b= m1->getTransform(state);
    BOOST_REQUIRE_CLOSE( m1_t3d_b.P()[0], m1_t3d.P()[0] , 1e-6);
    BOOST_REQUIRE_CLOSE( m1_t3d_b.P()[1], m1_t3d.P()[1] , 1e-6);
    BOOST_REQUIRE_CLOSE( m1_t3d_b.P()[2], m1_t3d.P()[2] , 1e-6);
    

    Frame::iterator_pair iter = world->getChildren();
    for(;iter.first!=iter.second; ++iter.first)
        std::cout << "c:" << (*(iter.first)).getID();
    std::cout << std::endl;
    iter = world->getChildren(state);
    for(;iter.first!=iter.second; ++iter.first)
        std::cout << "c:" << (*(iter.first)).getID();
    std::cout << std::endl;
    
    //std::cout << "Nr of children: " << children.size() << std::endl;
    //std::cout << "Nr of DAF children: " << dafchildren.size() << std::endl;
    
    BOOST_MESSAGE("- Testing Kinematic utils");
    std::vector<Frame*> frames = Kinematics::findAllFrames(world,state);
    
    
    
}


void singleChainTest()
{
    BOOST_MESSAGE("KinematicsTestSuite");
    BOOST_MESSAGE("Entering single chain test");
    
     
    FixedFrame* l1 = new FixedFrame("l1", Transform3D<>(Vector3D<>(1,2,3), RPY<>(0,0,0)));
    FixedFrame* l2 = new FixedFrame("l2", Transform3D<>(Vector3D<>(2,3,4), RPY<>(0,0,0)));
    FixedFrame* l3 = new FixedFrame("l3", Transform3D<>(Vector3D<>(3,4,5), RPY<>(0,0,0)));

    BOOST_MESSAGE("Creating StateStructure");
    boost::shared_ptr<StateStructure> tree( new StateStructure() );
    Frame* world = tree->getRoot();
    //tree->addFrame(world);
    BOOST_MESSAGE("Adding frames to tree");
    tree->addFrame(l1,world);
    tree->addFrame(l2,l1);
    tree->addFrame(l3,l2);
    
    BOOST_MESSAGE("Getting default state");
    State state = tree->getDefaultState();
    BOOST_MESSAGE("Calculating forward kinematics");
    Transform3D<> transform = Kinematics::frameTframe(world, l3, state);
    BOOST_MESSAGE("Testing results");
    BOOST_REQUIRE(transform.P()(0) == 6.0);
    BOOST_REQUIRE(transform.P()(1) == 9.0);
    BOOST_REQUIRE(transform.P()(2) == 12.0);
    
}

void multipleChainTest(){

    FixedFrame* l1 = new FixedFrame("l1", Transform3D<>(Vector3D<>(1,2,3), RPY<>(0,0,0)));
    FixedFrame* l2 = new FixedFrame("l2", Transform3D<>(Vector3D<>(2,3,4), RPY<>(0,0,0)));

    boost::shared_ptr<StateStructure> tree( new StateStructure() );
    Frame* world = tree->getRoot();
    tree->addFrame(l1,world);
    tree->addFrame(l2,world);

    //State state(tree);
    State state = tree->getDefaultState();
    Transform3D<> transform = Kinematics::frameTframe(world, l1, state);
    BOOST_REQUIRE(transform.P()(0) == 1.0);
    BOOST_REQUIRE(transform.P()(1) == 2.0);
    BOOST_REQUIRE(transform.P()(2) == 3.0);

    transform = Kinematics::frameTframe(world, l2, state);

    BOOST_REQUIRE(transform.P()(0) == 2.0);
    BOOST_REQUIRE(transform.P()(1) == 3.0);
    BOOST_REQUIRE(transform.P()(2) == 4.0);
    
}

KinematicsTestSuite::KinematicsTestSuite() :
    boost::unit_test::test_suite("KinematicsTestSuite")
{
    add( BOOST_TEST_CASE( &StateStructureTest ) );
    add( BOOST_TEST_CASE( &singleChainTest) );
    add( BOOST_TEST_CASE( &multipleChainTest) );
    
}
