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
