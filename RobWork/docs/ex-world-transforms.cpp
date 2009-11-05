#include <rw/kinematics/FKTable.hpp>
#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

#include <vector>
#include <boost/foreach.hpp>

std::vector<Transform3D<> > worldTransforms(
    const std::vector<Frame*>& frames, const State& state)
{
    FKTable fk(state);

    std::vector<Transform3D<> > result;
    BOOST_FOREACH(Frame* f, frames) {
        result.push_back(fk.get(*f));
    }
    return result;
}
