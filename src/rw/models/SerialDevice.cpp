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

#include <rw/common/macros.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::common;

namespace
{
    bool isActiveJoint(const Frame& frame)
    {
        return Accessor::ActiveJoint().has(frame);
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
            getKinematicChain(
                first, last, state)),
        state),

    _kinematicChain(
        getKinematicChain(
            first, last, state))
{}

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

const std::vector<Frame*>& SerialDevice::frames() const
{ return _kinematicChain; }
