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

#include <rwlibs/drawable/Drawable.hpp>
#include <rwlibs/drawable/RenderSTL.hpp>
#include <rwlibs/drawable/RenderAC3D.hpp>
#include <rwlibs/drawable/DrawableFactory.hpp>
#include <rwlibs/drawable/WorkCellGLDrawer.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/FixedFrame.hpp>

#include <rw/models/Accessor.hpp>

#include <string>
#include <fstream>

#include <boost/test/unit_test.hpp>

using namespace boost::unit_test;

using namespace rw::models;
using namespace rwlibs::drawable;
using namespace rw::math;
using namespace rw::kinematics;

BOOST_AUTO_TEST_CASE( testLoading ){
    BOOST_MESSAGE("- testing loading");
    RenderSTL stlaObject(testFilePath() + "chair.stla");
    RenderSTL stlbObject(testFilePath() + "cube.stlb");

    // test loading AC3D file
    RenderAC3D ac3dObject(testFilePath() + "Environment.ac");

    // test loading from input stream
    std::string infile(testFilePath() + "Environment.ac");
    std::ifstream in(infile.c_str());
    BOOST_REQUIRE(in.is_open());

    RenderAC3D ac3dObject2(in);
}

BOOST_AUTO_TEST_CASE(testDrawableFactory)
{
    BOOST_MESSAGE("- testing DrawableFactory");
    // test ascii stl format load
    Drawable* stlaObject = DrawableFactory::loadDrawableFile(testFilePath() + "chair");
    Drawable* stlbObject = DrawableFactory::loadDrawableFile(testFilePath() + "cube");
    //Drawable* p3dsObject = DrawableFactory::loadDrawableFile("exam");
    Drawable* ac3dObject = DrawableFactory::loadDrawableFile(testFilePath() + "Environment");

    stlaObject->setHighlighted(true);
    stlbObject->setHighlighted(true);
    //p3dsObject->setHighlighted(true);
    ac3dObject->setHighlighted(true);

    delete stlaObject;
    delete stlbObject;
    //delete p3dsObject;
    delete ac3dObject;
}

BOOST_AUTO_TEST_CASE(testWorkCellGLDrawer){
    BOOST_MESSAGE("- testing workcellGLDrawer");
    WorkCellGLDrawer workCellGLDrawer;

    const std::string filename = testFilePath() + "cube";

    FixedFrame* object1 = new FixedFrame("Object1", Transform3D<>::identity());
    FixedFrame* object2 = new FixedFrame("Object2", Transform3D<>::identity());

    StateStructure tree;
    Frame *world = tree.getRoot();
    tree.addFrame(object1,world);
    tree.addFrame(object2,world);

    DrawableModelInfo info(filename);
    Accessor::drawableModelInfo().set(*object1, std::vector<DrawableModelInfo>(1,info) );
    Accessor::drawableModelInfo().set(*object2, std::vector<DrawableModelInfo>(1,info) );

    //object1->getPropertyMap().set<std::string>("DrawableID", filename);
    //object2->getPropertyMap().set<std::string>("DrawableID", filename);
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
