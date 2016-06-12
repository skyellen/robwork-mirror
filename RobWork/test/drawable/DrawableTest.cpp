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

#include <rwlibs/opengl/DrawableFactory.hpp>

//#include <rw/kinematics/Frame.hpp>
//#include <rw/kinematics/StateStructure.hpp>
//#include <rw/kinematics/FixedFrame.hpp>

//#include <string>
//#include <fstream>

using namespace boost::unit_test;

//using namespace rw::graphics;
using rwlibs::opengl::DrawableFactory;
//using namespace rw::math;
//using namespace rw::kinematics;

BOOST_AUTO_TEST_CASE( testSTLLoading ){
	BOOST_TEST_MESSAGE("- testing loading");
    // test loading stl file
    rwlibs::opengl::Drawable::Ptr stlaObject =
    		DrawableFactory::loadDrawableFile( testFilePath() + "geoms/chair.stla", "chair" );
    rwlibs::opengl::Drawable::Ptr stlbObject =
    		DrawableFactory::loadDrawableFile( testFilePath() + "geoms/cube.stlb", "cube" );
}

BOOST_AUTO_TEST_CASE( testAC3DLoading ){
    // test loading AC3D file
    rwlibs::opengl::Drawable::Ptr ac3dObject =
    		DrawableFactory::loadDrawableFile(testFilePath() + "geoms/Gantry0.ac", "gantry");
    rwlibs::opengl::Drawable::Ptr ac3dObject1 =
            DrawableFactory::loadDrawableFile(testFilePath() + "geoms/Gantry0.ac3d", "gantry1");
}

BOOST_AUTO_TEST_CASE( testOBJLoading ){
    // test loading OBJ file
    rwlibs::opengl::Drawable::Ptr objObject =
            DrawableFactory::loadDrawableFile(testFilePath() + "geoms/fod1.obj", "fod1");
}

BOOST_AUTO_TEST_CASE( testTRILoading ){
    // test loading TRI file
    rwlibs::opengl::Drawable::Ptr objObject =
            DrawableFactory::loadDrawableFile(testFilePath() + "geoms/Rob-0.tri", "rob-0");
}

BOOST_AUTO_TEST_CASE( test3DSLoading ){
    // test loading 3ds file
    rwlibs::opengl::Drawable::Ptr objObject =
            DrawableFactory::loadDrawableFile(testFilePath() + "geoms/motor.3ds", "motor");
}

/*
BOOST_AUTO_TEST_CASE( testIVGLoading ){
    // test loading 3ds file
    rwlibs::opengl::Drawable::Ptr objObject =
            DrawableFactory::loadDrawableFile(testFilePath() + "geoms/staubli0.ivg", "staubli");
}
*/

BOOST_AUTO_TEST_CASE(testDrawableFactory)
{
	BOOST_TEST_MESSAGE("- testing DrawableFactory");
    // test ascii stl format load
    rwlibs::opengl::Drawable::Ptr stlaObject = DrawableFactory::loadDrawableFile(testFilePath() + "geoms/chair", "chair3");
    rwlibs::opengl::Drawable::Ptr stlbObject = DrawableFactory::loadDrawableFile(testFilePath() + "geoms/cube", "cube3");
    //rwlibs::drawable::Drawable* p3dsObject = DrawableFactory::loadDrawableFile("exam");
    rwlibs::opengl::Drawable::Ptr ac3dObject = DrawableFactory::loadDrawableFile(testFilePath() + "geoms/Environment","environment");

    stlaObject->setHighlighted(true);
    stlbObject->setHighlighted(true);
    //p3dsObject->setHighlighted(true);
    ac3dObject->setHighlighted(true);
}
/*
BOOST_AUTO_TEST_CASE(testWorkCellGLDrawer){
    BOOST_MESSAGE("- testing workcellGLDrawer");

    const std::string filename = testFilePath() + "cube";

    FixedFrame* object1 = new FixedFrame("Object1", Transform3D<>::identity());
    FixedFrame* object2 = new FixedFrame("Object2", Transform3D<>::identity());

    StateStructure tree;
    Frame *world = tree.getRoot();
    tree.addFrame(object1,world);
    tree.addFrame(object2,world);

    DrawableModelInfo info(filename,"mydrawable");
    DrawableModelInfo::set(std::vector<DrawableModelInfo>(1,info), object1);
    DrawableModelInfo::set(std::vector<DrawableModelInfo>(1,info), object2);

    Drawable::Ptr drawable1 = DrawableFactory::loadDrawableFile(filename,"mydrawable1");
    BOOST_REQUIRE(drawable1 != NULL);
    Drawable::Ptr drawable2 = DrawableFactory::loadDrawableFile(filename,"mydrawable2");
    BOOST_REQUIRE(drawable2 != NULL);
}
*/
