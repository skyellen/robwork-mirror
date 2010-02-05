#include "Accessor.hpp"

#include <rw/kinematics/FramePropertyImpl.hpp>

using namespace dynamics;

using namespace rw::models;
using namespace rw::kinematics;

const FrameProperty<RigidBodyInfo>& Accessor::RigidBodyInfo()
{
    static FramePropertyImpl<rw::models::RigidBodyInfo>
            accessor("RigidBodyInfo", "Info of rigid body properties");
    return accessor;
}
