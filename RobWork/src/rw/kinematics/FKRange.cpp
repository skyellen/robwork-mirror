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
    Transform3D<> reversePathTransform(const std::vector<const Frame*>& frames,
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
            (**p).multiplyTransform(transforms[pos], state, transforms[nextPos]);
            pos = nextPos;
        }
        return transforms[pos];


        // above has shown to be inefficient (JAR)
        /*
        RW_ASSERT(!frames.empty());
        typedef std::vector<const Frame*>::const_reverse_iterator I;
        const I end = frames.rend();

        I p = frames.rbegin();
        Transform3D<> result = (**p).getTransform(state);

        int pos = 0;
        for (++p; p != end; ++p) {
            result = result * (**p).getTransform(state);
        }
        return result;

        */





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

FKRange::FKRange() {

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

Frame::CPtr FKRange::getEnd() const
{
    if(_forwardBranch.empty()){
        return nullptr;
    }
    return _forwardBranch.front();
}

Frame::CPtr FKRange::getBase() const
{
    if(_inverseBranch.empty()){
        return nullptr;
    }
    return _inverseBranch.front();
}
