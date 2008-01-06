#include "TrajectoryPlanner.hpp"

#include <rw/math/Math.hpp>

using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::common;



TrajectoryPlanner::~TrajectoryPlanner()
{}

PropertyMap& TrajectoryPlanner::getProperties()
{
    return _properties;
}

const PropertyMap& TrajectoryPlanner::getProperties() const
{
    return _properties;
}
