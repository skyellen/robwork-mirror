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
#include <string>

#include "../TestSuiteConfig.hpp"

#include <boost/test/unit_test.hpp>

#include <rw/kinematics/FKTable.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rwsim/control/SerialDeviceController.hpp>

using rw::common::ownedPtr;
using namespace rw::kinematics;
using rw::math::Q;
using rw::models::Device;
using rwsim::loaders::DynamicWorkCellLoader;
using rwsim::simulator::ODESimulator;
using rwsim::dynamics::DynamicWorkCell;
using rwsim::control::SerialDeviceController;

BOOST_AUTO_TEST_CASE( ODEControlDeviceTest )
{
    // add loading tests here
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( testFilePath() + "/ur_control_test_scene/cup_pg70_table.dwc.xml");

    SerialDeviceController::Ptr devctrl = dwc->findController<SerialDeviceController>("URController");
    Device::Ptr ur = dwc->getWorkcell()->findDevice("UR-6-85-5-A");
    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );

    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();

    FKTable table(state);

    // test that the control interface works
    odesim->initPhysics(state);
    Q target(6,0,-0.2,0,0,0,0);
    devctrl->movePTP( target, 100);
    for(int i=0; i<200; i++){
    	std::cout << i << ":";
    	odesim->step(0.01, state);
    	std::cout << ur->getQ(state) << std::endl;
    }

    RW_WARN("end");
}

/*
BOOST_AUTO_TEST_CASE( ODEControlDeviceTest_FC )
{
    // add loading tests here
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( testFilePath() + "/ur_control_test_scene/cup_pg70_table.dwc.xml");

    SerialDeviceController::Ptr devctrl = dwc->findController<SerialDeviceController>("URController");
    Device::Ptr ur = dwc->getWorkcell()->findDevice("UR-6-85-5-A");
    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );

    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();

    std::cout << "Name of robot: " << ur->getName() << std::endl;

    Q viatarget(6,-0.0314697,-2.08739,-1.41194,-1.18469,1.56,-0.14103);
    ur->setQ(viatarget, state);

    FKTable table(state);

    // test that the control interface works
    odesim->initPhysics(state);


    // now give a FT command
    std::string taskframe("UR-6-85-5-A");
    // the selection 1 means force control, 0 means position control
    float selection[6] = {0,0,0,0,0,0};
    // target transform
    //Transform3D<> t3d_target( Vector3D<>(0.669, -0.131,0.142), RPY<>(-1.462,0,-3.115).toRotation3D() );
    Transform3D<> t3d_target( Vector3D<>(0.7, -0.131,0.142), RPY<>(-1.462,0,-3.115).toRotation3D() );
    // target wrench, push with 10N in the direction of z-axis
    Wrench6D<> wrench_target = Wrench6D<>(0,0,10,0,0,0);
    // offset relative to refframe
    Rotation3D<> t3d_offset = RPY<>(0,0,0).toRotation3D();

    devctrl->moveLinFC(t3d_target,wrench_target,selection,"UR-6-85-5-A.TCP", t3d_offset);

    std::ofstream myfile;
    myfile.open("data-test2.txt");

    for(int i=0; i<1000; i++){
    	Transform3D<> tf = ur->baseTend(state);
    	myfile << odesim->getTime() << "\t" << tf.P()[0] << "\t" << tf.P()[1] << "\t" << tf.P()[2] << "\n";

    	std::cout << i << ":";
    	odesim->step(0.01, state);
    	std::cout << ur->getQ(state) << std::endl;
    }

    RW_WARN("end");
}
*/
/*
BOOST_AUTO_TEST_CASE( ODESingleObjectStabilityTest )
{
    // add loading tests here
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( testFilePath() + "/scene/simple/single_object.dwc.xml");
    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );
    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();

    RigidBody::Ptr body = dwc->findBody<RigidBody>("ObjectCup2");

    // test that the control interface works
    odesim->initPhysics(state);
    while(odesim->getTime()<300){
        odesim->step(0.001, state);

        Transform3D<> wTb = body->wTcom(state);
        RPY<> v = RPY<>(wTb.R());
        Vector3D<> avel = body->getAngVel(state)*Rad2Deg;
        Vector3D<> lvel = body->getLinVel(state);

        std::cout << odesim->getTime() << " ";
        std::cout << wTb.P()[0] << " "<< wTb.P()[1] << " " << wTb.P()[2] << " ";
        std::cout << Rad2Deg*v[0] << " "<< Rad2Deg*v[1] << " " << Rad2Deg*v[2] << " " ;
        std::cout << lvel[0] << " "<< lvel[1] << " " << lvel[2] << " " ;
        std::cout << avel[0] << " "<< avel[1] << " " << avel[2] << " " ;
        std::cout << "\n";
    }
}
*/

