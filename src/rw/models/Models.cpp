/*********************************************************************
 * RobWork Version 0.2
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

#include "Models.hpp"
#include <rw/kinematics/Kinematics.hpp>

#include <boost/foreach.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::trajectory;

#define NS rw::models::Models

std::vector<Frame*> NS::findAllFrames(
    const WorkCell& workcell)
{
    return Kinematics::findAllFrames(
        workcell.getWorldFrame(),
        workcell.getDefaultState());
}

namespace
{
    bool inOrder(double a, double b, double c, double tolerance)
    {
        return
            a - tolerance < b &&
            b < c + tolerance;
    }
}

bool NS::inBounds(
    double val,
    const Joint& joint,
    double tolerance)
{
    return inOrder(
        joint.getBounds().first,
        val,
        joint.getBounds().second,
        tolerance);
}

bool NS::inBounds(
    const Q& q,
    const std::pair<Q, Q>& bounds,
    double tolerance)
{
    RW_ASSERT(tolerance >= 0);

    const Q& a = bounds.first;
    const Q& b = bounds.second;

    const int len = a.size();
    for (int i = 0; i < len; i++) {
        const double val = q[i];
        if (!inOrder(a[i], val, b[i], tolerance))
            return false;
    }
    return true;
}

bool NS::inBounds(
    const Q& q, const Device& device, double tolerance)
{
    return inBounds(q, device.getBounds(), tolerance);
}

bool NS::inBounds(
    const State& state,
    const WorkCell& workcell,
    double tolerance)
{
    const std::vector<Frame*>& frames = findAllFrames(workcell);

    typedef std::vector<Frame*>::const_iterator I;
    for (I p = frames.begin(); p != frames.end(); ++p) {
        const Joint* joint = dynamic_cast<const Joint*>(*p);
        if (joint) {
            const double val = *joint->getQ(state);
            if (!inBounds(val, *joint, tolerance))
                return false;
        }
    }

    return true;
}

void NS::getStatePath(
    const Device& device,
    const QPath& path,
    const State& common_state,
    StatePath& result)
{
    State state = common_state;
    BOOST_FOREACH(const Q& q, path) {
        device.setQ(q, state);
        result.push_back(state);
    }
}

StatePath NS::getStatePath(
    const Device& device,
    const QPath& path,
    const State& common_state)
{
    StatePath result;
    getStatePath(device, path, common_state, result);
    return result;
}
