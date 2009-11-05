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


#include "CollisionSetup.hpp"
#include <boost/foreach.hpp>

using namespace rw::proximity;

CollisionSetup::CollisionSetup()
    : _excludeStaticPairs(false)
{}

CollisionSetup::CollisionSetup(
    const ProximityPairList& exclude)
    :
    _exclude(exclude),
    _excludeStaticPairs(false)
{}

CollisionSetup::CollisionSetup(const ProximityPairList& exclude,
                               const std::set<std::string>& volatileFrames,
                               bool excludeStaticPairs):
    _exclude(exclude),
    _volatileFrames(volatileFrames),
    _excludeStaticPairs(excludeStaticPairs)
{

}

bool CollisionSetup::isVolatile(
    const rw::kinematics::Frame& frame) const
{
    return _volatileFrames.find(frame.getName()) != _volatileFrames.end();
}

void CollisionSetup::merge(const CollisionSetup& b)
{
    _exclude.insert(_exclude.end(), b.getExcludeList().begin(), b.getExcludeList().end());

    _volatileFrames.insert(
        b._volatileFrames.begin(),
        b._volatileFrames.end());

    // NB: excludeStaticPairs is a global setting!
    _excludeStaticPairs = _excludeStaticPairs || b._excludeStaticPairs;
}

CollisionSetup CollisionSetup::merge(
    const CollisionSetup& a,
    const CollisionSetup& b)
{
    CollisionSetup result = a;
    result.merge(b);
    return result;
}
