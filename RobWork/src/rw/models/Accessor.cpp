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


#include "Accessor.hpp"

#include <rw/kinematics/FramePropertyImpl.hpp>

using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::models;

const FrameProperty<CollisionSetup>& Accessor::collisionSetup()
{
    static FramePropertyImpl<rw::proximity::CollisionSetup> accessor(
        "CollisionSetup", "setup for collision checking");
    return accessor;
}

const FrameProperty<rw::kinematics::FrameType>& Accessor::frameType()
{
    static FramePropertyImpl<rw::kinematics::FrameType> accessor(
        "FrameType", "the type of frame");
    return accessor;
}

const FrameProperty<bool>& Accessor::activeJoint()
{
    static FramePropertyImpl<bool> accessor(
        "ActiveJoint", "an active joint");
    return accessor;
}

const FrameProperty<bool>& Accessor::dependentJoint()
{
    static FramePropertyImpl<bool> accessor(
        "DependentJoint", "an dependent joint");
    return accessor;
}

const FrameProperty<std::vector<DrawableModelInfo> >& Accessor::drawableModelInfo()
{
    static FramePropertyImpl<std::vector<DrawableModelInfo> > accessor(
        "DrawableModelInfo", "ID for the Drawable");
    return accessor;
}

const FrameProperty<std::vector<CollisionModelInfo> >& Accessor::collisionModelInfo()
{
    static FramePropertyImpl<std::vector<CollisionModelInfo> > accessor(
        "CollisionModelInfo", "ID for the Collision Model");
    return accessor;
}

const FrameProperty<DHParameterSet>& Accessor::dhSet()
{
    static FramePropertyImpl<DHParameterSet> accessor("DHSet", "Denavit-Hartenberg parameters");
    return accessor;
}
