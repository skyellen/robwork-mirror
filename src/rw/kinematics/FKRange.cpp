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

#include "FKRange.hpp"

#include <rw/common/macros.hpp>
#include "Frame.hpp"

using namespace rw::kinematics;
using namespace rw::math;

namespace
{
    // 'parent' is included. The path starts at 'parent' and goes to the root.
    std::vector<const Frame*> rootPath(const Frame* parent, const State& state)
    {
        std::vector<const Frame*> result;
        while (parent) {
            result.push_back(parent);
            parent = parent->getParent(state);
        }
        return result;
    }

    // The product of getTransform() values of the path in reverse order. The
    // path must be non-empty.
    Transform3D<> reversePathTransform(
        const std::vector<const Frame*>& frames,
        const State& state)
    {
        RW_ASSERT(!frames.empty());
        typedef std::vector<const Frame*>::const_reverse_iterator I;
        const I end = frames.rend();

        I p = frames.rbegin();
        Transform3D<> transforms[] = {
            (**p).getTransform(state),
            Transform3D<>()
        };
        int pos = 0;
        for (++p; p != end; ++p) {
            const int nextPos = 1 - pos;
            (**p).getTransform(transforms[pos], state, transforms[nextPos]);
            pos = nextPos;
        }
        return transforms[pos];

        /*
          It is not safe to call getTransform(cur, state, cur). 1. and 3.
          argument must be different objects.

        I p = frames.rbegin();
        Transform3D<> cur = (**p).getTransform(state);
        for (++p; p != end; ++p) {
            (**p).getTransform(cur, state, cur);
        }
        return cur;
        */

        /*
        I p = frames.rbegin();
        Transform3D<> result = (*p)->getTransform(state);

        for (++p; p != end; ++p) {
            result = result * (*p)->getTransform(state);
        }
        return result;
        */
    }
}

FKRange::FKRange(const Frame* from, const Frame* to, const State& state)
{
    // We allow a NULL-pointer to mean the world frame, so this check no longer
    // applies.
    //   RW_ASSERT(from && to);
    _inverseBranch = rootPath(from, state);
    _forwardBranch = rootPath(to, state);
    while (!_inverseBranch.empty() && !_forwardBranch.empty()) {
        if (_inverseBranch.back() == _forwardBranch.back()) {
            _inverseBranch.pop_back();
            _forwardBranch.pop_back();
        } else
            break;
    }
}

Transform3D<> FKRange::get(const State& state) const
{
    // These matrix operations _can_ be speeded up. For example some copying of
    // values can be omitted. We will deal with that later.
    if (_forwardBranch.empty() && _inverseBranch.empty()){
        return Transform3D<>::identity();
    } if (_inverseBranch.empty()){
        return reversePathTransform(_forwardBranch, state);
    }else if (_forwardBranch.empty()){
        return inverse(reversePathTransform(_inverseBranch, state));
    } else {
        return
            inverse(reversePathTransform(_inverseBranch, state)) *
            reversePathTransform(_forwardBranch, state);
    }
}
