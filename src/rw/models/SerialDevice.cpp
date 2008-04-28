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

#include <rw/math/Vector3D.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::common;

namespace
{
    bool isActiveJoint(const Frame& frame)
    {
        return Accessor::activeJoint().has(frame);
    }

    std::vector<Joint*> getActiveJoints(const std::vector<Frame*>& frames)
    {
        std::vector<Joint*> active;

        typedef std::vector<Frame*>::const_iterator I;
        for (I p = frames.begin(); p != frames.end(); ++p) {
            Frame* frame = *p;

            // But how do we know that isActiveJoint() implies Joint*? Why don't
            // we use a dynamic cast here for safety?
            if (isActiveJoint(*frame))
                active.push_back(static_cast<Joint*>(frame));
        }
        return active;
    }

    // From the root 'first' to the child 'last' inclusive.
    std::vector<Frame*> getChain(Frame* first, Frame* last, const State& state)
    {
        std::vector<Frame*> init =
            Kinematics::parentToChildChain(first, last, state);

        init.push_back(last);
        return init;
    }
}

SerialDevice::SerialDevice(
    Frame* first,
    Frame* last,
    const std::string& name,
    const State& state)
    :
    JointDevice(
        name,
        first,
        last,
        getActiveJoints(
            getChain(first, last, state)),
        state),

    _kinematicChain(
        getChain(first, last, state))
{}

const std::vector<Frame*>& SerialDevice::frames() const
{ return _kinematicChain; }

SerialDevice::SerialDevice(
    const std::vector<Frame*>& serialChain,
    const std::string& name,
    const State& state)
    :
    JointDevice(
        name,
        serialChain.front(),
        serialChain.back(),
        getActiveJoints(serialChain),
        state),

    _kinematicChain(serialChain)
{}
