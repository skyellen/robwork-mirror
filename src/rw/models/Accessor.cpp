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
