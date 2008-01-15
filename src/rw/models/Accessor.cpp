#include "Accessor.hpp"

#include <rw/kinematics/FramePropertyImpl.hpp>

using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::models;

const FrameProperty<CollisionSetup>& Accessor::CollisionSetup()
{
    static FramePropertyImpl<rw::proximity::CollisionSetup> accessor(
        "CollisionSetup", "setup for collision checking");
    return accessor;
}

const FrameProperty<rw::kinematics::FrameType>& Accessor::FrameType()
{
    static FramePropertyImpl<rw::kinematics::FrameType> accessor(
        "FrameType", "the type of frame");
    return accessor;
}

const FrameProperty<double>& Accessor::GeoScale()
{
    static FramePropertyImpl<double> accessor(
        "GeoScale", "scaling factor for CAD geometry");
    return accessor;
}

const FrameProperty<bool>& Accessor::ActiveJoint()
{
    static FramePropertyImpl<bool> accessor(
        "ActiveJoint", "an active joint");
    return accessor;
}

const FrameProperty<std::string>& Accessor::DrawableID()
{
    static FramePropertyImpl<std::string> accessor(
        "DrawableID", "ID for the Drawable");
    return accessor;
}

const FrameProperty<std::string>& Accessor::CollisionModelID()
{
    static FramePropertyImpl<std::string> accessor(
        "CollisionModelID", "ID for the Collision Model");
    return accessor;
}
