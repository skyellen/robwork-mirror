#include <rw/kinematics/FKRange.hpp>

using rw::math::Transform3D;
using namespace rw::kinematics;

Transform3D<> frameToFrameTransform(
    const Frame& a, const Frame& b, const State& state)
{
    FKRange fk(&a, &b, state);
    return fk.get(state);
}
