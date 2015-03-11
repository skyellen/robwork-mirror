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

#include <rw/rw.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;

#include "../TestSuiteConfig.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>

using namespace rwsim::loaders;
using namespace rwsim::simulator;
using namespace rwsim::dynamics;

BOOST_AUTO_TEST_CASE( SimpleDropObjectTest )
{
    // add loading tests here
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( testFilePath() + "/scene/simple/single_object.dwc.xml");

    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );
    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();
    RigidBody::Ptr body = dwc->findBody<RigidBody>("Tray");

    // test that the control interface works
    odesim->initPhysics(state);
    BOOST_CHECK_EQUAL(odesim->getTime(), 0.0 );

    // simulate for 10 sec.
    rw::common::Timer timer;
    double simTimeSec = 4;
    double dt = 0.01;
    while(odesim->getTime()<simTimeSec){
    	odesim->step(dt, state);
    	// break if this takes more than 20sec
    	if(timer.getTimeSec()>20)
    		break;
    }
    // check simulation time
    BOOST_CHECK_MESSAGE( odesim->getTime() >simTimeSec, "Sim time: " << odesim->getTime() );

    // what is it that we need to check... well, object should not have dropped through the table
    BOOST_CHECK_MESSAGE( body->wTbf(state).P()[2] >0.0, "object fell through floor..." );

    // also check if object is jumping around
    Vector3D<> startPos = body->wTbf(state).P();
    while(odesim->getTime()<simTimeSec+10){
    	odesim->step(dt, state);
    	if(timer.getTimeSec()>40)
    		break;
    }
    double distToStart = (body->wTbf(state).P()-startPos).norm2();
    BOOST_CHECK_MESSAGE( distToStart<0.005, "Object moved too far during 10 sec sim: " << (body->wTbf(state).P()-startPos));
}

BOOST_AUTO_TEST_CASE( OnTrayObjectStabilityTest )
{
    // add loading tests here
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( testFilePath() + "/scene/simple/cup_on_tray_stability.dwc.xml");
    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );
    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();

    RigidBody::Ptr body = dwc->findBody<RigidBody>("Cup");

    // test that the control interface works
    odesim->initPhysics(state);
    BOOST_CHECK_EQUAL(odesim->getTime(), 0.0 );

    rw::common::Timer timer;
    double dt = 0.01;

    // now first let stuff settle
    while(odesim->getTime()<4){
    	odesim->step(dt, state);
    	// break if this takes more than 20sec
    	if(timer.getTimeSec()>20)
    		break;
    }

    // what is it that we need to check... well, object should not have dropped through the table
    BOOST_CHECK_MESSAGE( body->wTbf(state).P()[2] >0.0, "object fell through floor..." );
    std::ofstream logFile;
    logFile.open("OnTrayObjectStabilityTestLog.txt");
    logFile << "time x y z rz ry rx vx vy vz ax ay az\n";
    // now register state of object and do a longer simulation
    Vector3D<> startPos = body->wTbf(state).P();
    while(odesim->getTime()<60){
    	odesim->step(dt, state);
    	// break if this takes more than 100sec
    	if(timer.getTimeSec()>100)
    		break;
    	{ // stuff for logging/printing/debugging
            Transform3D<> wTb = body->wTcom(state);
            RPY<> v = RPY<>(wTb.R());
            Vector3D<> avel = body->getAngVel(state)*Rad2Deg;
            Vector3D<> lvel = body->getLinVel(state);

            logFile << odesim->getTime() << " ";
            logFile << wTb.P()[0] << " "<< wTb.P()[1] << " " << wTb.P()[2] << " ";
            logFile << Rad2Deg*v[0] << " "<< Rad2Deg*v[1] << " " << Rad2Deg*v[2] << " " ;
            logFile << lvel[0] << " "<< lvel[1] << " " << lvel[2] << " " ;
            logFile << avel[0] << " "<< avel[1] << " " << avel[2] << " " ;
            logFile << "\n";
    	}
    }
    // what is it that we need to check... well, object should not have dropped through the table
    BOOST_CHECK_MESSAGE( body->wTbf(state).P()[2] >0.0, "object fell through floor..." );

    double distToStart = (body->wTbf(state).P()-startPos).norm2();
    BOOST_CHECK_MESSAGE( distToStart<0.005, "Object moved too far during 10 sec sim: " << (body->wTbf(state).P()-startPos));
}

BOOST_AUTO_TEST_CASE( MultiOnTrayObjectStabilityTest )
{
    // add loading tests here
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( testFilePath() + "/scene/simple/cups_on_tray_stability.dwc.xml");
    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );
    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();

    RigidBody::Ptr body = dwc->findBody<RigidBody>("Cup1");

    // test that the control interface works
    odesim->initPhysics(state);
    BOOST_CHECK_EQUAL(odesim->getTime(), 0.0 );

    rw::common::Timer timer;
    double dt = 0.005;

    // now first let stuff settle
    while(odesim->getTime()<4){
    	odesim->step(dt, state);
    	//std::cout << odesim->getTime() << std::endl;
    	// break if this takes more than 20sec
    	if(timer.getTimeSec()>20)
    		break;
    }

    // what is it that we need to check... well, object should not have dropped through the table
    BOOST_CHECK_MESSAGE( body->wTbf(state).P()[2] >0.0, "object fell through floor..." );
    std::ofstream logFile;
    logFile.open("MultiOnTrayObjectStabilityTestLog.txt");
    logFile << "time x y z rz ry rx vx vy vz ax ay az\n";
    // now register state of object and do a longer simulation
    Vector3D<> startPos = body->wTbf(state).P();
    while(odesim->getTime()<60){
    	odesim->step(dt, state);
    	// break if this takes more than 100sec
    	if(timer.getTimeSec()>100)
    		break;
    	{ // stuff for logging/printing/debugging
            Transform3D<> wTb = body->wTcom(state);
            RPY<> v = RPY<>(wTb.R());
            Vector3D<> avel = body->getAngVel(state)*Rad2Deg;
            Vector3D<> lvel = body->getLinVel(state);

            logFile << odesim->getTime() << " ";
            logFile << wTb.P()[0] << " "<< wTb.P()[1] << " " << wTb.P()[2] << " ";
            logFile << Rad2Deg*v[0] << " "<< Rad2Deg*v[1] << " " << Rad2Deg*v[2] << " " ;
            logFile << lvel[0] << " "<< lvel[1] << " " << lvel[2] << " " ;
            logFile << avel[0] << " "<< avel[1] << " " << avel[2] << " " ;
            logFile << "\n";
    	}
    }
    // what is it that we need to check... well, object should not have dropped through the table
    BOOST_CHECK_MESSAGE( body->wTbf(state).P()[2] >0.0, "object fell through floor..." );

    double distToStart = (body->wTbf(state).P()-startPos).norm2();
    BOOST_CHECK_MESSAGE( distToStart<0.005, "Object moved too far during 10 sec sim: " << (body->wTbf(state).P()-startPos));
}

