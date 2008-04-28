#include "MinimumClearanceCalculator.hpp"

#include <rw/common/Timer.hpp>
#include <rw/models/Accessor.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/proximity/DistanceCalculator.hpp>

using namespace rwlibs::pathoptimization;
using namespace rwlibs::proximitystrategies;
using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::common;

namespace
{
    boost::shared_ptr<DistanceCalculator> getDistanceCalculator(
        WorkCell* workcell,
        const State& state)
    {
        return boost::shared_ptr<DistanceCalculator>(
            new DistanceCalculator(
                workcell->getWorldFrame(),
                Accessor::collisionSetup().get(
                    *workcell->getWorldFrame()),
                new ProximityStrategyPQP(),
                state));
    }
}

MinimumClearanceCalculator::MinimumClearanceCalculator(
    boost::shared_ptr<DistanceCalculator> distancecalculator)
    :
    _distancecalculator(distancecalculator)
{}

MinimumClearanceCalculator::MinimumClearanceCalculator(
    WorkCell* workcell,
    const State& state)
    :
    _distancecalculator(getDistanceCalculator(workcell, state))
{}

MinimumClearanceCalculator::~MinimumClearanceCalculator()
{}

double MinimumClearanceCalculator::clearance(State& state)
{
    DistanceResult result = _distancecalculator->distance(state);
    return result.distance;
}
