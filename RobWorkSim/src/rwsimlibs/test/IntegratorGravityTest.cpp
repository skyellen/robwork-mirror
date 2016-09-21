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

#include "IntegratorGravityTest.hpp"

#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

#include <boost/bind.hpp>

using rw::common::PropertyMap;
using rw::kinematics::State;
using namespace rw::math;
using rw::trajectory::TimedState;
using rwsim::dynamics::DynamicWorkCell;
using rwsim::dynamics::RigidBody;
using rwsimlibs::test::IntegratorGravityTest;

IntegratorGravityTest::IntegratorGravityTest() {
}

IntegratorGravityTest::~IntegratorGravityTest() {
}

void IntegratorGravityTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	static const TestCallback cb( boost::bind(&IntegratorGravityTest::updateResults, _1) );
	const double dt = parameters.get<double>("Timestep")/1000.;

	// Initialize results with descriptions
	handle->addResult("Position in z",		"The z-coordinate of the object.");
	handle->addResult("Expected position",	"The analytical correct position.");
	handle->addResult("Deviation",			"The deviation from the expected position.");
	handle->addResult("Energy",				"The energy of the object.");
	handle->addResult("Velocity in xy",		"The velocity in xy.");
	handle->addResult("Velocity in z",		"The z-coordinate of the velocity.");
	handle->addResult("Angular Velocity",	"The angular velocity.");

	// Run standard loop
	runEngineLoop(dt,handle,engineID,parameters,verbose,cb);
}

double IntegratorGravityTest::getRunTime() const {
	return 1.;
}

void IntegratorGravityTest::updateResults(const EngineLoopInfo& info) {
	// Extract info
	const TestHandle::Ptr handle = info.handle;
	const std::string& engineID = info.engineID;
	const rw::common::Ptr<const DynamicWorkCell> dwc = info.dwc;
	const State& state = *info.state;
	const double time = info.time;
	const double dt = info.dt;

	// Get required info
	const RigidBody::Ptr rbody = dwc->findBody<RigidBody>("Object");
	RW_ASSERT(!rbody.isNull());
	const Transform3D<> T = rbody->getTransformW(state);
	const VelocityScrew6D<> vel = rbody->getVelocity(state);
	const double steps = Math::round(time/dt); // the type is double as int can not be assumed to be large enough
	const double gravity = dwc->getGravity()[2];

	// Calculate result values
	const double posZ = T.P()[2];
	const double posZexpected = 0.5*gravity*time*time;
	const double deviation = posZexpected-T.P()[2];
	const double energy = rbody->calcEnergy(state,dwc->getGravity());
	const double velXY = (vel.linear()-vel.linear()[2]*Vector3D<>::z()).norm2();
	const double velZ = vel.linear()[2];
	const double velAng = vel.angular().angle();

	// Add results
	handle->append(TimedState(time,state));
	handle->getResult("Position in z").addValue(time,posZ);
	handle->getResult("Expected position").addValue(time,posZexpected);
	handle->getResult("Deviation").addValue(time,deviation);
	handle->getResult("Energy").addValue(time,energy);
	handle->getResult("Velocity in xy").addValue(time,velXY);
	handle->getResult("Velocity in z").addValue(time,velZ);
	handle->getResult("Angular Velocity").addValue(time,velAng);

	// Add failures if results were not as expected
	const double eulerStepError = 0.5*gravity*dt*dt;
	if (time > 0) {
		if (engineID == "RWPEIsland")
			handle->getResult("Deviation").checkLastValues(eulerStepError,1e-14);
		else if (engineID == "Bullet")
			handle->getResult("Deviation").checkLastValues(-eulerStepError*steps,1e-5);
		else
			handle->getResult("Deviation").checkLastValues(-eulerStepError*steps,1e-14); // ODE
	}
	if (time > 0) {
		if (engineID == "RWPEIsland")
			handle->getResult("Energy").checkLastValues(rbody->getMass()*gravity*eulerStepError,1e-12);
		else if (engineID == "Bullet")
			handle->getResult("Energy").checkLastValues(-rbody->getMass()*gravity*eulerStepError*steps,1e-4);
		else
			handle->getResult("Energy").checkLastValues(-rbody->getMass()*gravity*eulerStepError*steps,1e-12); // ODE
	}
	handle->getResult("Velocity in xy").checkLastValues(0);
	handle->getResult("Velocity in z").checkLastValues(gravity*time,1e-5);
	handle->getResult("Angular Velocity").checkLastValues(0);
}
