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


#include "JointDevice.hpp"
#include "Joint.hpp"
#include "JointDeviceJacobianCalculator.hpp"

#include <rw/math/Jacobian.hpp>
//#include <rw/kinematics/FKTable.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::common;

// Constructor

namespace {
    size_t calcDOFs(const std::vector<Joint*>& joints) {
        size_t dof = 0;
        for (std::vector<Joint*>::const_iterator it = joints.begin(); it != joints.end(); ++it) {
            dof += (*it)->getDOF();
        }
        return dof;
    }

}

JointDevice::JointDevice(const std::string& name,
                         Frame* base,
                         Frame* end,
                         const std::vector<Joint*>& joints,
                         const State& state) :
    Device(name),
    _base(base),
    _end(end),
    _joints(joints),
    _dof(calcDOFs(joints)),
    _baseJCend(baseJCframe(end, state))
{


}

// Methods specific to JointDevice.
/*
Joint* JointDevice::getActiveJoint(size_t index) const
{
    const Joint* joint = _joints.at(index);
    return const_cast<Joint*>(joint);
}
*/
// Jacobians

Jacobian JointDevice::baseJend(const State& state) const
{
   /* FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * _dj.get(fk);
    */
    return _baseJCend->get(state);
    //return Jacobian(6,_dof);
}


/*
Jacobian JointDevice::baseJframes(const std::vector<Frame*>& frames,
                                  const State& state) const
{
    return baseDJframes(frames, state)->get(state);
}
*/
JacobianCalculatorPtr JointDevice::baseJCframes(const std::vector<Frame*>& frames,
                                                const State& state) const
{
    JacobianCalculator* jc = new JointDeviceJacobianCalculator(const_cast<JointDevice*>(this), _base, frames, state);
    return ownedPtr(jc);

}

// The rest is just forwarding to BasicDevice.

size_t JointDevice::getDOF() const
{
    return _dof;
}

std::pair<Q, Q> JointDevice::getBounds() const
{
    const Q q(getDOF());
    std::pair<Q, Q> bounds(q, q);
    int i = 0;
    for (std::vector<Joint*>::const_iterator p = _joints.begin(); p != _joints.end(); ++p) {
        const std::pair<Q, Q> pair = (*p)->getBounds();
        bounds.first.setSubPart(i,pair.first);
        bounds.second.setSubPart(i, pair.second);
        i += (*p)->getDOF();
    }

    return bounds;

}

void JointDevice::setBounds(const std::pair<Q, Q>& bounds)
{
    RW_ASSERT(getDOF() == bounds.first.size() && getDOF() == bounds.second.size());

    int i = 0;
    for (std::vector<Joint*>::iterator p = _joints.begin(); p != _joints.end(); ++p) {
        //p->setBounds(std::make_pair(bounds.first[i], bounds.second[i]));
		int dof = (*p)->getDOF();
		if (dof > 0) {
			(*p)->setBounds(std::make_pair(bounds.first.getSubPart(i, dof), bounds.second.getSubPart(i, dof)));
			i += dof;
		}
    }

}





void JointDevice::setQ(const Q& q, State& state) const
{
    if (q.size() != getDOF())
        RW_THROW("setQ() called for device of size "<< (int)getDOF()<< " and q of size "<< (int)q.size());

    int i = 0;
    for (std::vector<Joint*>::const_iterator it = _joints.begin(); it != _joints.end(); ++it) {
		if ((*it)->getDOF() > 0) {
			(*it)->setData(state, &(q[i]));
			i += (*it)->getDOF();
		}
    }
}

Q JointDevice::getQ(const State& state) const {
    Q q(getDOF());

    int i = 0;
    for (std::vector<Joint*>::const_iterator it = _joints.begin(); it != _joints.end(); ++it) {
        Q tmp((*it)->getDOF(), (*it)->getData(state));
        q.setSubPart(i, tmp);
        i += (*it)->getDOF();
    }
    return q;

}

Q JointDevice::getVelocityLimits() const {
    Q limits(getDOF());

    int i = 0;
    for (std::vector<Joint*>::const_iterator p = _joints.begin(); p != _joints.end(); ++p) {
        limits.setSubPart(i, (*p)->getMaxVelocity());
        i += (*p)->getDOF();
    }
    return limits;
}

void JointDevice::setVelocityLimits(const Q& vellimits)
{
    RW_ASSERT(getDOF() == vellimits.size());

    int i = 0;
    for (std::vector<Joint*>::iterator p = _joints.begin(); p != _joints.end(); ++p) {
        (*p)->setMaxVelocity(vellimits.getSubPart(i, (*p)->getDOF()));
        i += (*p)->getDOF();
    }
}

Q JointDevice::getAccelerationLimits() const
{
    Q limits(getDOF());

    int i = 0;
    for (std::vector<Joint*>::const_iterator p = _joints.begin(); p != _joints.end(); ++p) {
        limits.setSubPart(i, (*p)->getMaxAcceleration());
        i += (*p)->getDOF();
    }

    return limits;
}

void JointDevice::setAccelerationLimits(const Q& acclimits)
{
    RW_ASSERT(getDOF() == acclimits.size());

    int i = 0;
    for (std::vector<Joint*>::iterator p = _joints.begin(); p != _joints.end(); ++p, ++i) {
        (*p)->setMaxAcceleration(acclimits.getSubPart(i, (*p)->getDOF()));
        i += (*p)->getDOF();
    }
}
