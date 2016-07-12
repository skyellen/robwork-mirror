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

#include <rw/loaders/Model3DFactory.hpp>

#include <string>

using rw::graphics::Model3D;
using rw::loaders::Model3DFactory;

BOOST_AUTO_TEST_CASE( testSTLLoading ){
	BOOST_TEST_MESSAGE("- testing loading");
    // test loading stl file
    Model3D::Ptr stlaObject =
    		Model3DFactory::loadModel( testFilePath() + "geoms/chair.stla", "chair" );
    Model3D::Ptr stlbObject =
    		Model3DFactory::loadModel( testFilePath() + "geoms/cube.stlb", "cube" );
}

BOOST_AUTO_TEST_CASE( testAC3DLoading ){
    // test loading AC3D file
    Model3D::Ptr ac3dObject =
            Model3DFactory::loadModel(testFilePath() + "geoms/Gantry0.ac", "gantry");
    Model3D::Ptr ac3dObject1 =
            Model3DFactory::loadModel(testFilePath() + "geoms/Gantry0.ac3d", "gantry1");
}

BOOST_AUTO_TEST_CASE( testOBJLoading ){
    // test loading OBJ file
    Model3D::Ptr objObject =
            Model3DFactory::loadModel(testFilePath() + "geoms/fod1.obj", "fod1");
}

BOOST_AUTO_TEST_CASE( testTRILoading ){
    // test loading TRI file
    Model3D::Ptr objObject =
            Model3DFactory::loadModel(testFilePath() + "geoms/Rob-0.tri", "rob-0");
}

BOOST_AUTO_TEST_CASE( test3DSLoading ){
    // test loading 3ds file
    Model3D::Ptr objObject =
            Model3DFactory::loadModel(testFilePath() + "geoms/motor.3ds", "motor");
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
    Model3D::Ptr stlaObject = Model3DFactory::loadModel(testFilePath() + "geoms/chair", "chair3");
    Model3D::Ptr stlbObject = Model3DFactory::loadModel(testFilePath() + "geoms/cube", "cube3");
    Model3D::Ptr ac3dObject = Model3DFactory::loadModel(testFilePath() + "geoms/Environment","environment");

}
