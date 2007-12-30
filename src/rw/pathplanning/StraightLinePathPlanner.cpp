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

#include "StraightLinePathPlanner.hpp"

using namespace rw;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::models;
using namespace rw::pathplanning;

StraightLinePathPlanner::StraightLinePathPlanner(Device* device,
                                                 const State& state,
                                                 CollisionDetector* detector,
                                                 double resolution):
    utils(device, state, detector),
    _device(device),
    _resolution(resolution),
    _collisionChecks(0)
{}

bool StraightLinePathPlanner::interpolateMethod(const Q& start, const Q& end) const
{
    Q tmp1 = (end - start);
    Q delta = (tmp1 / (end - start).norm2() ) * _resolution;
    Q pos = start;
    while ( (end - pos).norm2() > _resolution){        
        if(inCollision(pos))
            return false;
        pos += delta;
    }

    return true;
}

bool StraightLinePathPlanner::inCollision(const Q& q) const {
    ++_collisionChecks;
    utils.inCollision(q);
}

bool StraightLinePathPlanner::query(const Q& qInit, const Q& qGoal, Path& path, double)
{
    if (!_device)
        return false;

    if (testQStart() && inCollision(qInit))
        return false;

    if(testQGoal() && inCollision(qGoal))
        return false;

    if(interpolateMethod(qInit, qGoal)==true){
        path.push_front(qInit);
        path.push_back(qGoal);
        return true;
    }

    return false;
}


