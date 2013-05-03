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

/**
 * This file tests the force torque sensor
 */

#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>

#include <rw/rw.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;


#include "../TestSuiteConfig.hpp"

#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rwsim/control/SerialDeviceController.hpp>

using namespace rwsim::loaders;
using namespace rwsim::simulator;
using namespace rwsim::dynamics;
using namespace rwsim::control;

BOOST_AUTO_TEST_CASE( ODEControlDeviceTest )
{
    // add loading tests here
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( testFilePath() + "/ur_control_test_scene/cup_pg70_table.dwc.xml");

    SerialDeviceController::Ptr devctrl = dwc->findController<SerialDeviceController>("URController");
    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );

    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();

    FKTable table(state);

    // test that the control interface works
    odesim->initPhysics(state);
    for(int i=0; i<1; i++){
    	RW_WARN("bum1");
    	odesim->step(0.001, state);
    	RW_WARN("bum2");
    }

    RW_WARN("end");
}
