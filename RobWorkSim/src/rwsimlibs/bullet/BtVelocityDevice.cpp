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

#include "BtVelocityDevice.hpp"

#include <rwsim/dynamics/RigidDevice.hpp>

#include <bullet/BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <bullet/BulletDynamics/ConstraintSolver/btSliderConstraint.h>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::bullet;

BtVelocityDevice::BtVelocityDevice(rw::common::Ptr<RigidDevice> rdev, const std::vector<btTypedConstraint*>& constraints):
	_rdev(rdev),
	_constraints(constraints)
{
}

BtVelocityDevice::~BtVelocityDevice() {
}

void BtVelocityDevice::update(double dt, State& state){
    const Q velQ = _rdev->getJointVelocities(state);
    for(unsigned int i = 0; i<_constraints.size(); i++){
        const double vel = velQ[i];
        if(dynamic_cast<btHingeConstraint*>(_constraints[i])){
            btHingeConstraint* const hinge = dynamic_cast<btHingeConstraint*>(_constraints[i]);
            hinge->enableAngularMotor(true, vel, 20);
        } else if( dynamic_cast<btSliderConstraint*>(_constraints[i]) ){
            btSliderConstraint* const slider = dynamic_cast<btSliderConstraint*>(_constraints[i]);
            slider->setTargetLinMotorVelocity(vel);
        }

    }
}

void BtVelocityDevice::postUpdate(State& state) {
    Q q = _rdev->getModel().getQ(state);
    for(unsigned int i = 0; i<_constraints.size(); i++){
        if(dynamic_cast<btHingeConstraint*>(_constraints[i])){
            btHingeConstraint* const hinge = dynamic_cast<btHingeConstraint*>(_constraints[i]);
            q[i] = hinge->getHingeAngle();
        } else if( dynamic_cast<btSliderConstraint*>(_constraints[i]) ){
//                btSliderConstraint* slider = dynamic_cast<btSliderConstraint*>(_constraints[i]); // simat - not used so commented to suppress warning
        }
    }
    _rdev->getModel().setQ(q, state);
}
