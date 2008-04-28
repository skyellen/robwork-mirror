#include "DrawableTestSuite.hpp"

#include <rwlibs/drawable/Drawable.hpp>
#include <rwlibs/drawable/RenderSTL.hpp>
#include <rwlibs/drawable/RenderAC3D.hpp>
#include <rwlibs/drawable/DrawableFactory.hpp>
#include <rwlibs/drawable/WorkCellGLDrawer.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/Tree.hpp>
#include <rw/kinematics/FixedFrame.hpp>

#include <string>
#include <fstream>

using namespace boost::unit_test;

using namespace rwlibs::drawable;
using namespace rw::math;
using namespace rw::kinematics;

void testLoading(){
    RenderSTL stlaObject("testfiles/chair.stla");
    RenderSTL stlbObject("testfiles/cube.stlb");

    // test loading AC3D file
    RenderAC3D ac3dObject("testfiles/Environment.ac");

    // test loading from input stream
    std::ifstream in("testfiles/Environment.ac");
    BOOST_REQUIRE(in.is_open());

    RenderAC3D ac3dObject2(in);
}

void testDrawableFactory(){
    // test ascii stl format load
    Drawable* stlaObject = DrawableFactory::loadDrawableFile("testfiles/chair");
    Drawable* stlbObject = DrawableFactory::loadDrawableFile("testfiles/cube");
    //Drawable* p3dsObject = DrawableFactory::loadDrawableFile("testfiles/exam");
    Drawable* ac3dObject = DrawableFactory::loadDrawableFile("testfiles/Environment");


    stlaObject->setHighlighted(true);
    stlbObject->setHighlighted(true);
    //p3dsObject->setHighlighted(true);
    ac3dObject->setHighlighted(true);

    delete stlaObject;
    delete stlbObject;
    //delete p3dsObject;
    delete ac3dObject;
}

void testWorkCellGLDrawer(){
    WorkCellGLDrawer workCellGLDrawer;

    const std::string filename = "testfiles/cube";

    FixedFrame* world = new FixedFrame(NULL, "World", Transform3D<>::identity());
    FixedFrame* object1 = new FixedFrame(world, "Object1", Transform3D<>::identity());
    FixedFrame* object2 = new FixedFrame(world, "Object2", Transform3D<>::identity());

    Tree tree;
    tree.addFrame(world);
    tree.addFrame(object1);
    tree.addFrame(object2);

    object1->getPropertyMap().set<std::string>("DrawableID", filename);
    object2->getPropertyMap().set<std::string>("DrawableID", filename);
        //    geoIDAccessor().set(*object1, filename);
        //    geoIDAccessor().set(*object2, filename);

    /*
    Drawable* drawable1 = DrawableFactory::loadDrawableFile(filename);
    BOOST_REQUIRE(drawable1 != NULL);
    Drawable* drawable2 = DrawableFactory::loadDrawableFile(filename);
    BOOST_REQUIRE(drawable2 != NULL);

    drawer.addDrawableToFrame(&object1, drawable1);
    drawer.addDrawableToFrame(&object2, drawable2);
    */

    std::vector<Drawable*> copy1 = workCellGLDrawer.getDrawablesForFrame(object1);
    BOOST_CHECK(copy1.size() == 1);

    std::vector<Drawable*> copy2 = workCellGLDrawer.getDrawablesForFrame(object2);
    BOOST_CHECK(copy2.size() == 1);


    BOOST_CHECK(workCellGLDrawer.getDrawablesForFrame(world).size() == 0);
}

DrawableTestSuite::DrawableTestSuite() :
    boost::unit_test::test_suite("DrawableTestSuite")
{
    BOOST_MESSAGE("DrawableTestSuite");
    add( BOOST_TEST_CASE( &testLoading) );
    add( BOOST_TEST_CASE( &testDrawableFactory ) );
    add( BOOST_TEST_CASE( &testWorkCellGLDrawer ) );
}
