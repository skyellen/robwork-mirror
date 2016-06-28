/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "IntegratorRotationTest.hpp"

#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

#include <boost/bind.hpp>

using rw::common::PropertyMap;
using rw::kinematics::State;
using namespace rw::math;
using rw::trajectory::TimedState;
using rwsim::dynamics::DynamicWorkCell;
using rwsim::dynamics::RigidBody;
using rwsimlibs::test::IntegratorRotationTest;

#define ANGULAR_VELOCITY 4*Pi

IntegratorRotationTest::IntegratorRotationTest() {
}

IntegratorRotationTest::~IntegratorRotationTest() {
}

void IntegratorRotationTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	static const InitCallback initCb( boost::bind(&IntegratorRotationTest::initialize, _1, _2) );
	static const TestCallback cb( boost::bind(&IntegratorRotationTest::updateResults, _1) );
	const double dt = parameters.get<double>("Timestep")/1000.;

	// Initialize results with descriptions
	handle->addResult("Distance",			"The distance of the center of mass from initial position.");
	handle->addResult("Energy",				"The energy of the object.");
	handle->addResult("Angular Velocity",	"The size of the angular velocity.");

	// Run standard loop
	runEngineLoop(dt,handle,engineID,parameters,verbose,cb,initCb);
}

double IntegratorRotationTest::getRunTime() const {
	return 30;
}

DynamicWorkCell::Ptr IntegratorRotationTest::makeIntegratorDWC(const std::string& integratorType) {
	const DynamicWorkCell::Ptr dwc = IntegratorTest::makeIntegratorDWC(integratorType);
	dwc->setGravity(Vector3D<>::zero());
	return dwc;
}

double IntegratorRotationTest::getExpectedEnergy(const rw::common::Ptr<const DynamicWorkCell> dwc) {
	State state = dwc->getWorkcell()->getDefaultState();
	const RigidBody::Ptr rbody = dwc->findBody<RigidBody>("Object");
	RW_ASSERT(!rbody.isNull());
	rbody->setAngVelW(Vector3D<>(0,0,ANGULAR_VELOCITY),state);
	return rbody->calcEnergy(state);
}

void IntegratorRotationTest::initialize(rw::common::Ptr<const DynamicWorkCell> dwc, State& state) {
	const RigidBody::Ptr rbody = dwc->findBody<RigidBody>("Object");
	RW_ASSERT(!rbody.isNull());
	rbody->setAngVelW(Vector3D<>(0,0,ANGULAR_VELOCITY),state);
}

void IntegratorRotationTest::updateResults(const EngineLoopInfo& info) {
	// Extract info
	const TestHandle::Ptr handle = info.handle;
	const std::string& engineID = info.engineID;
	const rw::common::Ptr<const DynamicWorkCell> dwc = info.dwc;
	const State& state = *info.state;
	const double time = info.time;

	// Get required info
	const RigidBody::Ptr rbody = dwc->findBody<RigidBody>("Object");
	RW_ASSERT(!rbody.isNull());
	const Transform3D<> T = rbody->getTransformW(state);

	// Calculate result values
	const double distance = T.P().norm2();
	const double energy = rbody->calcEnergy(state);
	const double velAng = rbody->getAngVel(state).norm2();

	handle->addResult("Distance",			"The distance of the center of mass from initial position.");
	handle->addResult("Energy",				"The energy of the object.");
	handle->addResult("Angular Velocity",	"The size of the angular velocity.");

	// Add results
	handle->append(TimedState(time,state));
	handle->getResult("Distance").addValue(time,distance);
	handle->getResult("Energy").addValue(time,energy);
	handle->getResult("Angular Velocity").addValue(time,velAng);

	// Add failures if results were not as expected
	const double expEnergy = getExpectedEnergy(dwc);
	handle->getResult("Distance").checkLastValues(0);

	// For Bullet >= 2.82
	//handle->getResult("Energy").checkLastValuesBetween(expEnergy*0.325,expEnergy*1.0095); // lower: Bullet, higher: RWPE
	//handle->getResult("Angular Velocity").checkLastValuesBetween(ANGULAR_VELOCITY*0.705,ANGULAR_VELOCITY*1.0045); // lower: Bullet, higher: RWPE

	// For Bullet <= 2.81
	handle->getResult("Energy").checkLastValuesBetween(expEnergy*0.325,expEnergy*400); // lower: Bullet, higher: Bullet <= 2.81
	handle->getResult("Angular Velocity").checkLastValuesBetween(ANGULAR_VELOCITY*0.705,ANGULAR_VELOCITY*20); // lower: Bullet, higher: Bullet <= 2.81

	// Engine specific tests
	if (engineID == "ODE") {
		// ODE 0.14:
		//handle->getResult("Energy").checkLastValuesBetween(expEnergy*0.915,expEnergy);
		//handle->getResult("Angular Velocity").checkLastValuesBetween(ANGULAR_VELOCITY*0.815,ANGULAR_VELOCITY);
		// ODE 0.11.1:
		handle->getResult("Energy").checkLastValuesBetween(expEnergy*0.915,expEnergy*400);
		handle->getResult("Angular Velocity").checkLastValuesBetween(ANGULAR_VELOCITY*0.815,ANGULAR_VELOCITY*20);
	} else if (engineID == "Bullet") { // Bullet >= 2.82
		//handle->getResult("Energy").checkLastValuesBetween(expEnergy*0.325,expEnergy);
		//handle->getResult("Angular Velocity").checkLastValuesBetween(ANGULAR_VELOCITY*0.705,ANGULAR_VELOCITY);
	} else if (engineID == "RWPEIsland") {
		handle->getResult("Energy").checkLastValuesBetween(expEnergy*0.9985,expEnergy*1.0095);
		handle->getResult("Angular Velocity").checkLastValuesBetween(ANGULAR_VELOCITY,ANGULAR_VELOCITY*1.0045);
	}
}
