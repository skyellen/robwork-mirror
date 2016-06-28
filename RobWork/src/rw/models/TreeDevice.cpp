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


#include "TreeDevice.hpp"

#include "Joint.hpp"
#include "DependentJoint.hpp"


#include <rw/common/macros.hpp>
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
        const Joint *j = dynamic_cast<const Joint*>(&frame);
        if(j!=NULL)
            return j->isActive();
        return false;
    }

    bool isDependentJoint(const Frame& frame) {
        if( dynamic_cast<const DependentJoint*>(&frame)!=NULL )
            return true;
        return false;
    }

    std::vector<Joint*> getJointsFromFrames(const std::vector<Frame*>& frames)
    {
        std::vector<Joint*> active;

        typedef std::vector<Frame*>::const_iterator I;
        for (I p = frames.begin(); p != frames.end(); ++p) {
            Frame* frame = *p;

            // But how do we know that isActiveJoint() implies Joint*? Why don't
            // we use a dynamic cast here for safety?
            if (isActiveJoint(*frame) || isDependentJoint(*frame))
                active.push_back(static_cast<Joint*>(frame));
        }

        return active;
    }

    // From the root 'first' to the child 'last', but with 'last' excluded.
    std::vector<Frame*> getChain(Frame* first, Frame* last, const State& state)
    {
        return Kinematics::reverseChildToParentChain(last, first, state);
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
			std::vector<Frame*> chain = getChain(first, *p, state);
			BOOST_FOREACH(Frame* frame, chain) {
				std::vector<Frame*>::iterator it = std::find(kinematicChain.begin(), kinematicChain.end(), frame);
				if (it == kinematicChain.end())
					kinematicChain.push_back(frame);
			}
            //kinematicChain.insert(
            //    kinematicChain.end(), chain.begin(), chain.end());
        }
        return kinematicChain;
    }
}

TreeDevice::TreeDevice(Frame* base,
                       const std::vector<Frame*>& ends,
                       const std::string& name,
                       const State& state):
    JointDevice(name,
                base,
                ends.front(),
                getJointsFromFrames(getKinematicTree(base, ends, state)), state),

    _kinematicChain(getKinematicTree(base, ends, state)),

    _ends(ends),

    _djmulti(baseJCframes(ends, state))
{

}


TreeDevice::~TreeDevice(){

}

// Jacobians

Jacobian TreeDevice::baseJends(const State& state) const
{
    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    //return inverse(start.R()) * _djmulti->get(fk);
    return inverse(start.R()) * _djmulti->get(state);
}

