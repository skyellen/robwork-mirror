#include <rw/kinematics/FKRange.hpp>
#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

Transform3D<> frameToFrameTransform(
    const Frame& a, const Frame& b, const State& state)
{
    FKRange fk(&a, &b, state);
    return fk.get(state);
}
