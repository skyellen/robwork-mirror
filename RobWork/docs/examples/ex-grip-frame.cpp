#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/math/Transform3D.hpp>

using namespace rw::math;
using namespace rw::kinematics;

void gripMovableFrame(
    MovableFrame& item, Frame& gripper, State& state)
{
    FKRange fk(&gripper, &item, state);
    const Transform3D<> transform = fk.get(state);
    item.setTransform(transform, state);
    item.attachTo(&gripper, state);
}
