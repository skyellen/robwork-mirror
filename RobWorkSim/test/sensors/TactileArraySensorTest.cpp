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

#include "../TestSuiteConfig.hpp"

#include <RobWorkSimConfig.hpp>
#ifdef RWSIM_HAVE_ODE

#include <rw/loaders/path/PathLoader.hpp>
#include <rw/sensor/TactileArrayModel.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/sensor/TactileArraySensor.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>

using rw::common::ownedPtr;
using rw::kinematics::State;
using rw::sensor::TactileArrayModel;
using namespace rw::trajectory;

using namespace rwsim::dynamics;
using rwsim::loaders::DynamicWorkCellLoader;
using rwsim::sensor::TactileArraySensor;
using rwsim::simulator::ODESimulator;

BOOST_AUTO_TEST_CASE( TactileArraySensorTest )
{
	// load a scene with a FT sensor mounted in between a kinematic body and a dynamic
	// place the kinematic body in different poses and pla
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::loader.load( testFilePath() + "/scene/sensors/single_object_tactile_array.dwc.xml" );

    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );
    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();

    KinematicBody::Ptr kbody = dwc->findBody<KinematicBody>("Floor");
    RigidBody::Ptr body = dwc->findBody<RigidBody>("Tray");

    TactileArraySensor::Ptr sensor = dwc->findSensor<TactileArraySensor>("FTArraySensor");
    BOOST_REQUIRE(!sensor.isNull());
    TimedStatePath tpath;
    // test that the control interface works
    odesim->initPhysics(state);
    BOOST_CHECK_EQUAL(odesim->getTime(), 0.0 );
    for(int i=0; i<1000; i++){
    	std::cout << i << "\t";
    	odesim->step(0.01, state);
    	// print the ft sensor readings
    	TactileArrayModel::ValueMatrix values = sensor->getTexelData(state);
    	std::cout << odesim->getTime() << "\t";
    	for(int x=0;x<values.rows();x++ )
    		for(int y=0;y<values.cols();y++ ){
    			std::cout << values(x,y) << "\t";
    		}
    	std::cout << std::endl;
    	tpath.push_back( TimedState(odesim->getTime(),state));

    }
    rw::loaders::PathLoader::storeTimedStatePath(*dwc->getWorkcell(), tpath,"tpath.rwplay");



}

#else
BOOST_AUTO_TEST_CASE( TactileArraySensorTest ) {
	BOOST_FAIL("Not compiled with ODE simulator.");
}
#endif // RWSIM_HAVE_ODE
