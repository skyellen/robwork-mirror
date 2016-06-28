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

#include "IntegratorSpringTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rwsim/dynamics/Constraint.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

#include <boost/bind.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using rw::models::WorkCell;
using rw::trajectory::TimedState;
using namespace rwsim::dynamics;
using rwsimlibs::test::IntegratorSpringTest;

#define GRAVITY -9.82
#define RADIUS 0.1

IntegratorSpringTest::IntegratorSpringTest() {
}

IntegratorSpringTest::~IntegratorSpringTest() {
}

void IntegratorSpringTest::run(TestHandle::Ptr handle, const std::string& engineID, const PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose) {
	static const TestCallback cb( boost::bind(&IntegratorSpringTest::updateResults, _1) );
	const double dt = parameters.get<double>("Timestep")/1000.;

	// Initialize results with descriptions
	handle->addResult("Position in z",		"The z-coordinate of the object.");
	handle->addResult("Expected position",	"The analytical correct position.");
	handle->addResult("Deviation",			"The deviation from the expected position.");
	handle->addResult("Energy",				"The total energy.");
	handle->addResult("Spring Energy",		"The spring energy.");
	handle->addResult("Velocity in xy",		"The velocity in xy.");
	handle->addResult("Velocity in z",		"The z-coordinate of the velocity.");
	handle->addResult("Expected velocity",	"The analytical correct velocity.");
	handle->addResult("Deviation velocity",	"The deviation from the expected velocity.");
	handle->addResult("Angular Velocity",	"The angular velocity.");

	// Run standard loop
	runEngineLoop(dt,handle,engineID,parameters,verbose,cb);
}

double IntegratorSpringTest::getRunTime() const {
	return 5;
}

DynamicWorkCell::Ptr IntegratorSpringTest::makeIntegratorDWC(const std::string& integratorType) {
	static const double r = RADIUS;

	const WorkCell::Ptr wc = ownedPtr(new WorkCell("IntegratorSpringTestWorkCell"));
	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));

	DynamicWorkCellBuilder builder;
	builder.addCylinderFixed(dwc,r*3/4,r/5);
	builder.addBall(dwc,r,7850);
	builder.addMaterialData(dwc,0,0);

	const Transform3D<> Tf(Vector3D<>::z()*r*1.2);
	wc->findFrame<FixedFrame>("Cylinder")->setTransform(Tf);

	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	wc->findFrame<MovableFrame>("Ball")->setTransform(Transform3D<>::identity(),state);
	stateStructure->setDefaultState(state);

	dwc->setGravity(Vector3D<>(0,0,GRAVITY));

	const Constraint::Ptr constraint = ownedPtr(new Constraint("Spring", Constraint::Free, dwc->findBody("Cylinder").get(), dwc->findBody("Ball").get()));
	Constraint::SpringParams spring;
	spring.enabled = true;
	spring.compliance = Eigen::MatrixXd::Zero(6,6);
	spring.compliance(0,0) = 0.001;
	spring.compliance(1,1) = 0.001;
	spring.compliance(2,2) = 0.001;
	spring.compliance(3,3) = 1;
	spring.compliance(4,4) = 1;
	spring.compliance(5,5) = 1;
	spring.damping = Eigen::MatrixXd::Zero(6,6);
	constraint->setSpringParams(spring);
	dwc->addConstraint(constraint);

	return dwc;
}

double IntegratorSpringTest::referencePosition(double t) {
	const double k = 1000.;
	const double m = RADIUS*RADIUS*RADIUS*Pi*4/3*7850;
	const double f = m*GRAVITY;
	const double pinit = 0.12; // already 120 mm extended
	return (f - (f+k*pinit)*cos((sqrt(k)*t)/sqrt(m)))/k+pinit;
}

double IntegratorSpringTest::referenceVelocity(double t) {
	const double k = 1000.;
	const double m = RADIUS*RADIUS*RADIUS*Pi*4/3*7850;
	const double f = m*GRAVITY;
	const double pinit = 0.12; // already 120 mm extended
	return (f/k+pinit)*sqrt(k/m)*sin(sqrt(k/m)*t);
}

void IntegratorSpringTest::updateResults(const EngineLoopInfo& info) {
	// Extract info
	const TestHandle::Ptr handle = info.handle;
	const std::string& engineID = info.engineID;
	const rw::common::Ptr<const DynamicWorkCell> dwc = info.dwc;
	const State& state = *info.state;
	const double time = info.time;

	// Get required info
	const RigidBody::Ptr rbody = dwc->findBody<RigidBody>("Ball");
	RW_ASSERT(!rbody.isNull());
	const Transform3D<> T = rbody->getTransformW(state);
	const VelocityScrew6D<> vel = rbody->getVelocity(state);

	// Calculate result values
	const double k = 1000.;
	const double posZ = T.P()[2];
	const double posZexp = referencePosition(time);
	const double deviation = posZexp-posZ;
	const double springEnergy = 0.5*k*(posZ-0.12)*(posZ-0.12);
	const double energy = rbody->calcEnergy(state,dwc->getGravity())+springEnergy;
	const double velXY = (vel.linear()-vel.linear()[2]*Vector3D<>::z()).norm2();
	const double velZ = vel.linear()[2];
	const double velZexp = referenceVelocity(time);
	const double deviationVel = velZexp-velZ;
	const double velAng = vel.angular().angle();

	// Add results
	handle->append(TimedState(time,state));
	handle->getResult("Position in z").addValue(time,posZ);
	handle->getResult("Expected position").addValue(time,posZexp);
	handle->getResult("Deviation").addValue(time,deviation);
	handle->getResult("Energy").addValue(time,energy);
	handle->getResult("Spring Energy").addValue(time,springEnergy);
	handle->getResult("Velocity in xy").addValue(time,velXY);
	handle->getResult("Velocity in z").addValue(time,velZ);
	handle->getResult("Expected velocity").addValue(time,velZexp);
	handle->getResult("Deviation velocity").addValue(time,deviationVel);
	handle->getResult("Angular Velocity").addValue(time,velAng);

	// Add failures if results were not as expected
	const double initialSpringEnergy = 0.5*k*0.12*0.12;
	handle->getResult("Deviation").checkLastValues(0,0.0065);
	handle->getResult("Energy").checkLastValues(initialSpringEnergy,initialSpringEnergy*0.085);
	handle->getResult("Velocity in xy").checkLastValues(0);
	handle->getResult("Deviation velocity").checkLastValues(0,0.004);
	handle->getResult("Angular Velocity").checkLastValues(0);

	// Engine specific tests
	if (engineID == "RWPEIsland") {
		handle->getResult("Deviation").checkLastValues(0,0.0008);
		handle->getResult("Energy").checkLastValues(initialSpringEnergy,initialSpringEnergy*0.009);
	}
}
