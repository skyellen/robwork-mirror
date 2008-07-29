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

#include "TrajectoryFactory.hpp"

//#include "PointTimeIndex.hpp"
//#include "PointTimeIndexFactory.hpp"

#include <rw/math/Q.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>


using namespace rw::trajectory;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

typedef Timed<State> TimedState;


Trajectory<State> TrajectoryFactory::makeLinearTrajectory(const StatePath& path)
{
    Trajectory<State> trajectory;

    if (path.size() > 1) {
        StatePath::const_iterator it1 = path.begin();
        StatePath::const_iterator it2 = it1; it2++;

        for (;it2 != path.end(); ++it1, ++it2) {
            LinearInterpolator<State>* interp = new LinearInterpolator<State>(*it1, *it2, 1);
            trajectory.add(interp);
        }
    } else if (path.size() == 1) {
        LinearInterpolator<State>* interp = new LinearInterpolator<State>(path.front(), path.front(), 1);
        trajectory.add(interp);
    }
    return trajectory;
}

Trajectory<State> TrajectoryFactory::makeLinearTrajectory(const TimedStatePath& path)
{
    Trajectory<State> trajectory;

    if (path.size() > 1) {
        TimedStatePath::const_iterator it1 = path.begin();
        TimedStatePath::const_iterator it2 = ++(path.begin());

        for (;it2 != path.end(); ++it1, ++it2) {
            LinearInterpolator<State>* interp = new LinearInterpolator<State>((*it1).getValue(), (*it2).getValue(), (*it2).getTime()-(*it1).getTime());
            trajectory.add(interp);
        }
    } else if (path.size() == 1) {
        LinearInterpolator<State>* interp = new LinearInterpolator<State>(path.front().getValue(), path.front().getValue(), 1);
        trajectory.add(interp);
    }
    return trajectory;
}

Trajectory<State> TrajectoryFactory::makeEmptyStateTrajectory()
{
    return Trajectory<State>();
}
