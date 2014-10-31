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


#include "FixedFrame.hpp"

#include <rw/kinematics/Kinematics.hpp>

using namespace rw::kinematics;
using namespace rw::math;

FixedFrame::FixedFrame(const std::string& name,
                       const Transform3D<>& transform) :
    Frame(0, name),
    _transform(transform)
{}


const Transform3D<>& FixedFrame::getFixedTransform() const
{
    return _transform;
}

void FixedFrame::doMultiplyTransform(const Transform3D<>& parent,
                                     const State& state,
                                     Transform3D<>& result) const
{
    Transform3D<>::multiply(parent, _transform, result);
}

void FixedFrame::setTransform(const Transform3D<>& transform){
	_transform = transform;
}


Transform3D<> FixedFrame::doGetTransform(const State& state) const {
    return _transform;
}

void FixedFrame::moveTo(const rw::math::Transform3D<>& refTtarget, Frame* refframe, State& state){
    // first calculate transform from refframe to parent frame
    Transform3D<> parentTref = Kinematics::frameTframe(getParent(), refframe, state);
    Transform3D<> parentTmframe = parentTref * refTtarget;
    setTransform( parentTmframe);
}
