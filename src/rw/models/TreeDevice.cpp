#include "TreeDevice.hpp"

#include "Joint.hpp"
#include "Accessor.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Jacobian.hpp>

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

    // From the root 'first' to the child 'last', but with 'last' excluded.
    std::vector<Frame*> getChain(Frame* first, Frame* last, const State& state)
    {
        return Kinematics::ReverseChildToParentChain(last, first, state);
    }

    std::vector<Frame*> getKinematicTree(
        Frame* first,
        const std::vector<Frame*>& last,
        const State& state)
    {
        RW_ASSERT(first);

        typedef std::vector<Frame*> V;
        typedef V::const_iterator I;

        V kinematicChain;
        kinematicChain.push_back(first);
        for (I p = last.begin(); p != last.end(); ++p) {
            const V chain = getChain(first, *p, state);
            kinematicChain.insert(
                kinematicChain.end(), chain.begin(), chain.end());
        }
        return kinematicChain;
    }
}

TreeDevice::TreeDevice(
    Frame* base,
    const std::vector<Frame*>& ends,
    const std::string& name,
    const State& state)
    :
    JointDevice(
        name,
        base,
        ends.front(),
        getActiveJoints(
            getKinematicTree(base, ends, state)),
        state),

    _kinematicChain(
        getKinematicTree(base, ends, state)),

    _ends(ends),

    _djmulti(
        baseJframes(ends, state))
{}

// Jacobians

Jacobian TreeDevice::baseJends(const State& state) const
{
    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * _djmulti->get(fk);
}
