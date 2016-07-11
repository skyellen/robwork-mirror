/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "RecursiveNewtonEuler.hpp"

#include <rw/math/InertiaMatrix.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rwsim::dynamics;
using namespace rwsim::util;

RecursiveNewtonEuler::RecursiveNewtonEuler(RigidDevice::Ptr device):
	_rdev(device),
	_jdev(device->getJointDevice()),
	_gravity(Vector3D<>::zero()),
	_payloadCOM(Vector3D<>::zero()),
	_payloadMass(0),
	_payloadInertia(InertiaMatrix<>()),
	_valid(validate())
{
}

RecursiveNewtonEuler::~RecursiveNewtonEuler() {
}

Vector3D<> RecursiveNewtonEuler::getGravity() const {
	return _gravity;
}

void RecursiveNewtonEuler::setGravity(const Vector3D<> &gravity) {
	_gravity = gravity;
}

Vector3D<> RecursiveNewtonEuler::getPayloadCOM() const {
	return _payloadCOM;
}

double RecursiveNewtonEuler::getPayloadMass() const {
	return _payloadMass;
}

InertiaMatrix<> RecursiveNewtonEuler::getPayloadInertia() const {
	return _payloadInertia;
}

void RecursiveNewtonEuler::setPayload(const Vector3D<> &com, double payload, const InertiaMatrix<> &inertia) {
	_payloadCOM = com;
	_payloadMass = payload;
	_payloadInertia = inertia;
}

Wrench6D<> RecursiveNewtonEuler::getEnvironment() const {
	return _environment;
}

void RecursiveNewtonEuler::setEnvironment(const Wrench6D<> &wrench) {
	_environment = wrench;
}

std::vector<RecursiveNewtonEuler::Motion> RecursiveNewtonEuler::getBodyMotion(const State &state, const Q &dq, const Q &ddq) const {
	if (!_valid)
		RW_THROW(invalidMsg());
	std::vector<Motion> res(_jdev->getDOF()+1);
	Transform3D<> baseTlink = Kinematics::frameTframe(_jdev->getBase(),_jdev->getJoints()[0]->getParent(),state);
	Vector3D<> z;
	Vector3D<> w, dw, v, dv = -_gravity; // acceleration must be opposite to the gravitational acceleration
	Vector3D<> tmp, tmp2;
	Vector3D<> dr, pCur;
	Vector3D<> rCOM, drCOM, ddrCOM;
	unsigned int e;
	for (std::size_t k = 1; k <= _jdev->getDOF(); k++) {
		Joint* joint = _jdev->getJoints()[k-1];
		pCur = baseTlink.P();
		if (k==1)
			baseTlink = baseTlink*joint->getTransform(state);
		else
			baseTlink = baseTlink*Kinematics::frameTframe(_jdev->getJoints()[k-2],joint,state);
		dr = baseTlink.P()-pCur;
		z = baseTlink.R().getCol(2);
		e = 0;
		if (dynamic_cast<PrismaticJoint*>(joint))
			e = 0;
		else if (dynamic_cast<RevoluteJoint*>(joint))
			e = 1;
		tmp = w;
		tmp2 = dw;
		w += e*dq[k-1]*z;
		dw += e*(ddq[k-1]*z+cross(tmp,dq[k-1]*z));
		v += cross(tmp,dr)+(1-e)*dq[k-1]*z;
		dv += cross(tmp2,dr)+cross(tmp,cross(tmp,dr))+(1-e)*(ddq[k-1]*z+2*cross(tmp,dq[k-1]*z));

		// Find linear velocity and acceleration around center of mass
		Body::Ptr link = _rdev->getLinks()[k-1];
		rCOM = baseTlink.R()*link->getInfo().masscenter;
		drCOM = v + cross(w,rCOM);
		ddrCOM = dv + cross(dw,rCOM) + cross(w,cross(w,rCOM));

		// Save result
		res[k-1].velocity = VelocityScrew6D<>(drCOM,EAA<>(w));
		res[k-1].acceleration = VelocityScrew6D<>(ddrCOM,EAA<>(dw));
	}
	// Find tool velocities and accelerations
	{
		pCur = baseTlink.P();
		baseTlink = baseTlink*Kinematics::frameTframe(_jdev->getJoints().back(),_jdev->getEnd(),state);
		dr = baseTlink.P()-pCur;
		v += cross(w,dr);
		dv += cross(dw,dr)+cross(w,cross(w,dr));

		// Find linear velocity and acceleration around center of mass
		rCOM = baseTlink.R()*_payloadCOM;
		drCOM = v + cross(w,rCOM);
		ddrCOM = dv + cross(dw,rCOM) + cross(w,cross(w,rCOM));

		// Save result
		res[_jdev->getDOF()].velocity = VelocityScrew6D<>(drCOM,EAA<>(w));
		res[_jdev->getDOF()].acceleration = VelocityScrew6D<>(ddrCOM,EAA<>(dw));
	}
	return res;
}

std::vector<Wrench6D<> > RecursiveNewtonEuler::getBodyNetForces(const std::vector<RecursiveNewtonEuler::Motion> &motions, const State &state) const {
	if (!_valid)
		RW_THROW(invalidMsg());
	std::vector<Wrench6D<> > res(_jdev->getDOF()+1);
	Transform3D<> baseTlink = Kinematics::frameTframe(_jdev->getBase(),_jdev->getJoints()[0]->getParent(),state);
	for (std::size_t i = 0; i < _jdev->getDOF(); i++) {
		Motion motion = motions[i];
		Body::Ptr link = _rdev->getLinks()[i];
		Joint* joint = _jdev->getJoints()[i];
		if (i==0)
			baseTlink = baseTlink*joint->getTransform(state);
		else
			baseTlink = baseTlink*Kinematics::frameTframe(_jdev->getJoints()[i-1],joint,state);
		InertiaMatrix<> inertia = baseTlink.R()*link->getInertia()*inverse(baseTlink.R());
		double mass = link->getInfo().mass;
		const VelocityScrew6D<> &vel = motion.velocity;
		const VelocityScrew6D<> &acc = motion.acceleration;
		Vector3D<> velAng(vel.angular().axis()*vel.angular().angle());
		Vector3D<> accAng(acc.angular().axis()*acc.angular().angle());
		Vector3D<> force = mass*acc.linear();
		Vector3D<> torque = inertia*accAng+cross(velAng,inertia*velAng);
		res[i] = Wrench6D<>(force,torque);
	}
	// Handle tool
	{
		baseTlink = Kinematics::frameTframe(_jdev->getBase(),_jdev->getEnd(),state);
		Motion motion = motions[_jdev->getDOF()];
		const VelocityScrew6D<> &vel = motion.velocity;
		const VelocityScrew6D<> &acc = motion.acceleration;
		Vector3D<> velAng(vel.angular().axis()*vel.angular().angle());
		Vector3D<> accAng(acc.angular().axis()*acc.angular().angle());
		Vector3D<> force = _payloadMass*acc.linear();
		InertiaMatrix<> inertia = baseTlink.R()*_payloadInertia*inverse(baseTlink.R());
		Vector3D<> torque = inertia*accAng+cross(velAng,inertia*velAng);
		res[_jdev->getDOF()] = Wrench6D<>(force,torque);
	}
	return res;
}

std::vector<Wrench6D<> > RecursiveNewtonEuler::getJointForces(const std::vector<Wrench6D<> > &forces, const State &state) const {
	std::vector<Wrench6D<> > res(_jdev->getDOF()+1);
	// Handle tool
	Transform3D<> baseTlink = Kinematics::frameTframe(_jdev->getBase(),_jdev->getEnd(),state);
	{
		std::size_t k = _jdev->getDOF();
		Vector3D<> force = forces[k].force() + _environment.force();
		Vector3D<> torque = forces[k].torque() - cross(baseTlink.R()*_payloadCOM,force) + _environment.torque();
		res[k] = Wrench6D<>(force,torque);
	}
	Vector3D<> pNext;
	// Backward step
	for (std::size_t k = _jdev->getDOF(); k >= 1; k--) {
		Body::Ptr link = _rdev->getLinks()[k-1];
		pNext = baseTlink.P();
		baseTlink = Kinematics::frameTframe(_jdev->getBase(),_jdev->getJoints()[k-1],state);
		Vector3D<> com = baseTlink.R()*link->getInfo().masscenter;
		Vector3D<> dr = pNext-baseTlink.P();
		Vector3D<> force = forces[k-1].force() + res[k].force();
		Vector3D<> torque = forces[k-1].torque() + res[k].torque()+cross(com,force) + cross(dr-com,res[k].force());
		res[k-1] = Wrench6D<>(force,torque);
	}
	return res;
}

std::vector<Wrench6D<> > RecursiveNewtonEuler::solve(const State &state, const Q dq, const Q ddq) const {
	if (!_valid)
		RW_THROW(invalidMsg());
	std::vector<Motion> motions = getBodyMotion(state,dq,ddq);
	std::vector<Wrench6D<> > forces = getBodyNetForces(motions,state);
	return getJointForces(forces,state);
}

std::vector<double> RecursiveNewtonEuler::solveMotorTorques(const State &state, const Q &dq, const Q &ddq) const {
	if (!_valid)
		RW_THROW(invalidMsg());
	std::vector<Motion> motions = getBodyMotion(state,dq,ddq);
	std::vector<Wrench6D<> > forces = getBodyNetForces(motions,state);
	std::vector<Wrench6D<> > jointForces = getJointForces(forces,state);
	std::vector<double> res(_jdev->getDOF()+1);
	Transform3D<> baseTlink = Kinematics::frameTframe(_jdev->getBase(),_jdev->getJoints()[0]->getParent(),state);
	Vector3D<> z;
	for (std::size_t k = 0; k < res.size()-1; k++) {
		Joint* joint = _jdev->getJoints()[k];
		if (k == 0)
			baseTlink = baseTlink*joint->getTransform(state);
		else
			baseTlink = baseTlink*Kinematics::frameTframe(_jdev->getJoints()[k-1],joint,state);
		z = baseTlink.R().getCol(2);
		res[k] = dot(jointForces[k].torque(),z);
	}
	res[res.size()-1] = dot(jointForces[res.size()-1].torque(),z);
	return res;
}

bool RecursiveNewtonEuler::validate() const {
	bool valid = false;
	if (_jdev.cast<SerialDevice>())
		valid = true;
	if (!valid)
		return false;
	for (std::size_t i = 0; i < _jdev->getDOF(); i++) {
		valid = false;
		Joint* joint = _jdev->getJoints()[i];
		if (dynamic_cast<PrismaticJoint*>(joint))
			valid = true;
		if (dynamic_cast<RevoluteJoint*>(joint))
			valid = true;
		if (!valid)
			return false;
	}
	return true;
}

std::string RecursiveNewtonEuler::invalidMsg() {
	return "Recursive Newton-Euler only works for a SerialDevice and only for devices with prismatic or revolute joints.";
}

Vector3D<> RecursiveNewtonEuler::toVector3D(const EAA<> &eaa) {
	return Vector3D<>(eaa.axis()*eaa.angle());
}
