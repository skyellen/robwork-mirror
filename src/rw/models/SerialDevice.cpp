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

#include "SerialDevice.hpp"
#include "Accessor.hpp"
#include "Joint.hpp"
#include "RevoluteJoint.hpp"
#include "PrismaticJoint.hpp"
#include "JacobianUtil.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/kinematics/Kinematics.hpp>




using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::common;

//----------------------------------------------------------------------
// SerialDevice

namespace
{
    bool isActiveJoint(const Frame& frame)
    {
        return Accessor::ActiveJoint().has(frame);
    }

    std::vector<Joint*> getActiveJoints(const std::vector<Frame*>& frames)
    {
        // But how do we know that isActiveJoint() implies Joint*? Why don't we
        // use a dynamic cast here for safety?

        std::vector<Joint*> active;

        typedef std::vector<Frame*>::const_iterator I;
        for (I p = frames.begin(); p != frames.end(); ++p) {
            Frame* frame = *p;
            if (isActiveJoint(*frame))
                active.push_back((Joint*)frame);
        }

        return active;
    }

    // The chain of frames connecting \b first to \b last.
    // \b last is included in the list, but \b first is excluded.
    std::vector<Frame*> getKinematicChain(
        Frame* first,
        Frame* last,
        const State& state)
    {
        RW_ASSERT(first);
        RW_ASSERT(last);

        typedef std::vector<Frame*> V;

        V reverseChain;

        for (
            Frame* frame = last;
            frame != first;
            frame = frame->getParent(state))
        {
            if (!frame)
                RW_THROW(
                    "Frame "
                    << StringUtil::Quote(first->getName())
                    << " is not on the parent chain of "
                    << StringUtil::Quote(last->getName()));

            reverseChain.push_back(frame);
        }

        return V(reverseChain.rbegin(), reverseChain.rend());
    }
}

SerialDevice::SerialDevice(class Frame* first,
                           class Frame* last,
                           const std::string& name,
                           const State& state):
    Device(name),
    _base(first),
    _end(last),
    _kinematicChain(getKinematicChain(_base, _end, state)),
    _activeJoints(getActiveJoints(_kinematicChain)),
    _basicDevice(_activeJoints),
    _dj(_basicDevice, _end, state)
{}

SerialDevice::SerialDevice(
    const std::vector<Frame*>& serialChain,
    const std::string& name,
    const State& state)
    :
    Device(name),
    _base(serialChain.front()),
    _end(serialChain.back()),
    _kinematicChain(serialChain),
    _activeJoints(getActiveJoints(serialChain)),
    _basicDevice(_activeJoints),
    _dj(_basicDevice, _end, state)
{}

SerialDevice::~SerialDevice()
{}

//----------------------------------------------------------------------
// Jacobians for SerialDevice

Jacobian SerialDevice::baseJend(const State& state) const
{
    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * _dj.get(fk);
}

Jacobian SerialDevice::baseJframe(const Frame* frame, const State& state) const
{
    BasicDeviceJacobian dj(_basicDevice, frame, state);

    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * dj.get(fk);
}

boost::shared_ptr<BasicDeviceJacobian> 
SerialDevice::baseJframes(const std::vector<Frame*>& frames,
                            const State& state) const 
{   
    BasicDeviceJacobian *devjac = new BasicDeviceJacobian(_basicDevice, frames, state);
    return boost::shared_ptr<BasicDeviceJacobian>(devjac);
}

//----------------------------------------------------------------------
// The rest is just forwarding to BasicDevice.

std::pair<Q, Q> SerialDevice::getBounds() const
{
    return _basicDevice.getBounds();
}

void SerialDevice::setBounds(const std::pair<Q, Q>& bounds)
{
    return _basicDevice.setBounds(bounds);
}

void SerialDevice::setQ(const Q& q, State& state) const 
{
    _basicDevice.setQ(q, state);
}

Q SerialDevice::getQ(const State& state) const
{
    return _basicDevice.getQ(state);
}

Q SerialDevice::getVelocityLimits() const
{
    return _basicDevice.getVelocityLimits();
}

void SerialDevice::setVelocityLimits(const Q& vellimits)
{
    _basicDevice.setVelocityLimits(vellimits);
}

Q SerialDevice::getAccelerationLimits() const
{
    return _basicDevice.getAccelerationLimits();
}

void SerialDevice::setAccelerationLimits(const Q& q)
{
    return _basicDevice.setAccelerationLimits(q);
}
