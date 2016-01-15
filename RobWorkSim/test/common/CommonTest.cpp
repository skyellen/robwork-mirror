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

#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
using namespace rwsim::dynamics;
using namespace rwsim::loaders;

BOOST_AUTO_TEST_CASE( DynamicWorkCellLoaderTest )
{
    // check failed loading of non-existing scene
	BOOST_CHECK_THROW( DynamicWorkCellLoader::load(testFilePath() + "/devices/does_not_exist.dwc.xml"), std::exception);

	// check successfull loading of different dynamic workcells
    DynamicWorkCell::Ptr dwc_pg70, dwc_sdh, dwc_ur;
    BOOST_CHECK_NO_THROW( dwc_pg70 = DynamicWorkCellLoader::load(testFilePath() + "/devices/PG70/test_scene.dwc.xml") );
    BOOST_CHECK_NO_THROW( dwc_sdh = DynamicWorkCellLoader::load(testFilePath() + "/devices/SDH2/test_scene.dwc.xml") );
    BOOST_CHECK_NO_THROW( dwc_ur = DynamicWorkCellLoader::load(testFilePath() + "/devices/UR6855A/test_scene.dwc.xml") );

    // check that the scenes contain correct information
    BOOST_CHECK( dwc_pg70->findController("GraspController") != NULL );
    BOOST_CHECK( dwc_pg70->findDevice("PG70") != NULL );
    BOOST_CHECK( dwc_pg70->findDevice<RigidDevice>("PG70") != NULL );
    BOOST_CHECK( dwc_pg70->findBody("PG70.Base") != NULL );
    BOOST_CHECK( dwc_pg70->findBody("PG70.RightFinger") != NULL );

    // check different constraints
    DynamicWorkCell::Ptr fixed, prismatic, revolute, universal, spherical, piston, prismaticRotoid, prismaticUniversal, free, free_spring, revolute_limits;
    BOOST_CHECK_NO_THROW( fixed = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/fixed.dwc.xml") );
    BOOST_CHECK_NO_THROW( prismatic = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/prismatic.dwc.xml") );
    BOOST_CHECK_NO_THROW( revolute = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/revolute.dwc.xml") );
    BOOST_CHECK_NO_THROW( universal = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/universal.dwc.xml") );
    BOOST_CHECK_NO_THROW( spherical = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/spherical.dwc.xml") );
    BOOST_CHECK_NO_THROW( piston = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/piston.dwc.xml") );
    BOOST_CHECK_NO_THROW( prismaticRotoid = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/prismaticRotoid.dwc.xml") );
    BOOST_CHECK_NO_THROW( prismaticUniversal = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/prismaticUniversal.dwc.xml") );
    BOOST_CHECK_NO_THROW( free = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/free.dwc.xml") );
    BOOST_CHECK_NO_THROW( free_spring = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/free_spring.dwc.xml") );
    BOOST_CHECK_NO_THROW( revolute_limits = DynamicWorkCellLoader::load(testFilePath() + "/scene/constraints/revolute_limits.dwc.xml") );

    // check information
    BOOST_CHECK(!fixed->findConstraint("Constraint").isNull());
    BOOST_CHECK(!prismatic->findConstraint("Constraint").isNull());
    BOOST_CHECK(!revolute->findConstraint("Constraint").isNull());
    BOOST_CHECK(!universal->findConstraint("Constraint").isNull());
    BOOST_CHECK(!spherical->findConstraint("Constraint").isNull());
    BOOST_CHECK(!piston->findConstraint("Constraint").isNull());
    BOOST_CHECK(!prismaticRotoid->findConstraint("Constraint").isNull());
    BOOST_CHECK(!prismaticUniversal->findConstraint("Constraint").isNull());
    BOOST_CHECK(!free->findConstraint("Constraint").isNull());
    BOOST_CHECK(!free_spring->findConstraint("Constraint").isNull());
    BOOST_CHECK(!revolute_limits->findConstraint("Constraint").isNull());

    BOOST_CHECK(!prismatic->findConstraint("Constraint")->getSpringParams().enabled);
    BOOST_CHECK(free_spring->findConstraint("Constraint")->getSpringParams().enabled);
    BOOST_CHECK(!revolute->findConstraint("Constraint")->getLimit(0).lowOn);
    BOOST_CHECK(!revolute->findConstraint("Constraint")->getLimit(0).highOn);
    BOOST_CHECK(revolute_limits->findConstraint("Constraint")->getLimit(0).lowOn);
    BOOST_CHECK(revolute_limits->findConstraint("Constraint")->getLimit(0).highOn);
}

