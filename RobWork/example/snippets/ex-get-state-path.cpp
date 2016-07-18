#include <rw/models/Device.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>

#include <boost/foreach.hpp>
#include <vector>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

std::vector<State> getStatePath(
    const Device& device,
    const std::vector<Q>& path,
    const State& common_state)
{
    State state = common_state;

    std::vector<State> result;
    BOOST_FOREACH(const Q& q, path) {
        device.setQ(q, state);
        result.push_back(state);
    }
    return result;
}
