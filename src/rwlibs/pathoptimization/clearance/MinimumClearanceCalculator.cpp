/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


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
