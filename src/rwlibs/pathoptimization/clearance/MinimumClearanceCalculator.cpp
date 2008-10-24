/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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
    DistanceCalculatorPtr getDistanceCalculator(WorkCellPtr workcell,
                                                const State& state)
    {
        return ownedPtr(new DistanceCalculator(workcell->getWorldFrame(),
                                               Accessor::collisionSetup().get(*workcell->getWorldFrame()),
                                               new ProximityStrategyPQP(),
                                               state));
    }
}

MinimumClearanceCalculator::MinimumClearanceCalculator(DistanceCalculatorPtr distancecalculator)
    :
    _distancecalculator(distancecalculator)
{}

MinimumClearanceCalculator::MinimumClearanceCalculator(WorkCellPtr workcell,
                                                       const State& state):
    _distancecalculator(getDistanceCalculator(workcell, state))
{}

MinimumClearanceCalculator::~MinimumClearanceCalculator()
{}

double MinimumClearanceCalculator::clearance(State& state)
{
    DistanceResult result = _distancecalculator->distance(state);
    return result.distance;
}
