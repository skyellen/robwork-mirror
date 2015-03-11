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

#include <rwsim/rwsim.hpp>
#ifdef RWSIM_HAVE_ODE
#include <rwsimlibs/ode/ODESimulator.hpp>
#endif
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace rwsim::simulator;

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

}

