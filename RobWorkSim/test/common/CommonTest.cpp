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
    // add loading tests here
    DynamicWorkCell::Ptr dwc1 = DynamicWorkCellLoader::load(testFilePath() + "/simple/device1.dwc.xml");

    DynamicWorkCell::Ptr dwc2 = DynamicWorkCellLoader::load(testFilePath() + "/simple/device2.dwc.xml");

    DynamicWorkCell::Ptr dwc3 = DynamicWorkCellLoader::load(testFilePath() + "/simple/device3.dwc.xml");

    DynamicWorkCell::Ptr dwcM1 = DynamicWorkCellLoader::load(testFilePath() + "/simple/deviceMulti1.dwc.xml");

}


BOOST_AUTO_TEST_CASE( ODESimulatorLoadTest )
{
#ifdef RWSIM_HAVE_ODE
    // add loading tests here
    DynamicWorkCellLoader loader;
    DynamicWorkCell::Ptr dwc = loader.load("bumbum");
    ODESimulator *sim = new ODESimulator(dwc);
    sim->getGravity(); // Dummy
#else
	BOOST_FAIL("Simulator is not compiled with ODE - hence ODESimulator can not load workcell.");
#endif
}


BOOST_AUTO_TEST_CASE( ODESimulatorResetTest )
{
#ifdef RWSIM_HAVE_ODE
	// Test if the simulator can handle to be reset
    DynamicWorkCellLoader loader;
    DynamicWorkCell::Ptr dwc = loader.load(testFilePath() + "/simple/device1.dwc.xml");
    ODESimulator *sim = new ODESimulator(dwc);
    sim->getGravity(); // Dummy
#else
	BOOST_FAIL("Simulator is not compiled with ODE - hence ODESimulator can not be reset.");
#endif
}

