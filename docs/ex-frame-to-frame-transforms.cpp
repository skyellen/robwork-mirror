#include <rw/kinematics/FKRange.hpp>
#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

#include <boost/foreach.hpp>
#include <vector>

std::vector<Transform3D<> > frameToFrameTransforms(
    const Frame& a,
    const Frame& b,
    const State& tree_structure,
    const std::vector<State>& states)
{
    FKRange fk(&a, &b, tree_structure);

    std::vector<Transform3D<> > result;
    BOOST_FOREACH(const State& state, states) {
        result.push_back(fk.get(state));
    }
    return result;
}
