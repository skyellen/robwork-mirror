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

#include "PlannerUtil.hpp"

#include <rw/math/Math.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::pathplanning;


PlannerUtil::PlannerUtil(Device* device, const State& state, CollisionDetector* detector):
    _device(device),
    _state(state),
    _collisionDetector(detector)
{
}


PlannerUtil::~PlannerUtil() {
}

void PlannerUtil::setState(rw::kinematics::State& state) {
    _state = state;
}


Q PlannerUtil::randomConfig() const
{
    RW_ASSERT(NULL != _device);

    Q qn(_device->getDOF());
    for (size_t i = 0; i < qn.size(); i++)
        qn[i] = Math::Ran();

    return unNormalize(qn);
}

bool PlannerUtil::inCollision(const Q& q) const
{
    RW_ASSERT(NULL != _device);
    State state = _state;
    _device->setQ(q, state);

    return _collisionDetector->inCollision(state);
}

Q PlannerUtil::normalize(const Q& q) const
{
    RW_ASSERT(NULL != _device);

    Q qn(q.size());
    std::pair<Q, Q> bounds = _device->getBounds();
    for(size_t i = 0; i<qn.size(); i++){
        qn[i] = (q[i] - bounds.first[i]) / (bounds.second[i] - bounds.first[i]);
    }
    return qn;
}

Q PlannerUtil::unNormalize(const Q& qn) const
{
    RW_ASSERT(NULL != _device);

    Q q(qn.size());
    std::pair<Q, Q> bounds = _device->getBounds();
    for(size_t i = 0; i<q.size(); i++){
        q[i] = qn[i] * (bounds.second[i] - bounds.first[i]) + bounds.first[i];
    }
    return q;
}
