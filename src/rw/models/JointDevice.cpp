/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "JointDevice.hpp"
#include "Joint.hpp"

#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/FKTable.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::common;

// Constructor

JointDevice::JointDevice(
    const std::string& name,
    Frame* base,
    Frame* end,
    const std::vector<Joint*>& joints,
    const State& state)
    :
    Device(name),
    _base(base),
    _end(end),
    _bd(joints),
    _dj(_bd, _end, state)
{}

// Methods specific to JointDevice.

Joint* JointDevice::getActiveJoint(size_t index) const
{
    const Joint* joint = &getBasicDevice().at(index);
    return const_cast<Joint*>(joint);
}

boost::shared_ptr<BasicDeviceJacobian> 
JointDevice::baseJframes(
    const std::vector<Frame*>& frames, const State& state) const 
{   
    return boost::shared_ptr<BasicDeviceJacobian>(
        new BasicDeviceJacobian(getBasicDevice(), frames, state));
}

// Jacobians

Jacobian JointDevice::baseJend(const State& state) const
{
    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * _dj.get(fk);
}

Jacobian JointDevice::baseJframe(const Frame* frame, const State& state) const
{
    BasicDeviceJacobian dj(_bd, frame, state);
    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * dj.get(fk);
}

// The rest is just forwarding to BasicDevice.

size_t JointDevice::getDOF() const { return _bd.getDOF(); }

std::pair<Q, Q> JointDevice::getBounds() const { return _bd.getBounds(); }

void JointDevice::setBounds(const std::pair<Q, Q>& bounds)
{ return _bd.setBounds(bounds); }

void JointDevice::setQ(const Q& q, State& state) const { _bd.setQ(q, state); }

Q JointDevice::getQ(const State& state) const { return _bd.getQ(state); }

Q JointDevice::getVelocityLimits() const { return _bd.getVelocityLimits(); }

void JointDevice::setVelocityLimits(const Q& vellimits)
{ _bd.setVelocityLimits(vellimits); }

Q JointDevice::getAccelerationLimits() const
{ return _bd.getAccelerationLimits(); }

void JointDevice::setAccelerationLimits(const Q& q)
{ return _bd.setAccelerationLimits(q); }
