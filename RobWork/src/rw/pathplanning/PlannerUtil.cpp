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


#include "PlannerUtil.hpp"
#include "QSampler.hpp"

#include <rw/math/Math.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/math/Jacobian.hpp>
#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::pathplanning;

bool PlannerUtil::inCollision(
    const PlannerConstraint& constraint,
    const rw::math::Q& start,
    const rw::math::Q& end,
    bool checkStart,
    bool checkEnd)
{
    return
        (checkStart && constraint.getQConstraint().inCollision(start)) ||
        (checkEnd && constraint.getQConstraint().inCollision(end)) ||
        constraint.getQEdgeConstraint().inCollision(start, end);
}

bool PlannerUtil::inCollision(
    const PlannerConstraint& constraint,
    const std::vector<Q>& path)
{
    BOOST_FOREACH(const Q& q, path) {
        if (constraint.getQConstraint().inCollision(q))
            return true;
    }

    for (size_t i = 1; i < path.size(); ++i) {
        const Q& a = path[i - 1];
        const Q& b = path[i];
        if (constraint.getQEdgeConstraint().inCollision(a, b))
            return true;
    }

    return false;
}

namespace
{
    Q divide(double s, const Q& q)
    {
        Q result(q.size());
        for (size_t i = 0; i < q.size(); i++)
            result[i] = s / q[i];
        return result;
    }
}

QMetric::Ptr PlannerUtil::normalizingInfinityMetric(
    const Device::QBox& bounds,
    double length)
{
    return MetricFactory::makeWeightedInfinity(
        divide(length, bounds.second - bounds.first));
}

QMetric::Ptr PlannerUtil::timeMetric(const Q& speed)
{
    return MetricFactory::makeWeightedInfinity(divide(1, speed));
}

QMetric::Ptr PlannerUtil::timeMetric(const Device& device)
{
    return timeMetric(device.getVelocityLimits());
}

Q PlannerUtil::estimateMotionWeights(
    const Device& device,
    const Frame* frame,
    const State& initialState,
    EstimateType type,
    size_t samples)
{
	QSampler::Ptr sampler = QSampler::makeUniform(device);

    if (frame == NULL) frame = device.getEnd();

    size_t n = device.getDOF();
    State state = initialState;
    Q ws = Q(Q::zero(n));
    for (size_t i = 0; i < samples; i++) {
        Q q = sampler->sample();
        device.setQ(q, state);
        Jacobian jac = device.baseJframe(frame, state);
        for (size_t j = 0; j < n; j++) {
            double dx = jac(0,j);
            double dy = jac(1,j);
            double dz = jac(2,j);
            double w = sqrt(dx*dx + dy*dy + dz*dz);
            switch (type) {
            case WORSTCASE:
                if (w > ws(j))
                    ws(j) = w;
                break;
            case AVERAGE:
                ws(j) += w;
                break;
            }
        }
    }

    if (type == AVERAGE)
        ws /= samples;

    return ws;
}

rw::math::Q PlannerUtil::clampPosition(
    const Device::QBox& bounds,
    const rw::math::Q& q)
{
    RW_ASSERT(q.size() == bounds.first.size());

    Q res(q);
    for (size_t i = 0; i < q.size(); i++) {
        if (res(i) < bounds.first(i))
            res(i) = bounds.first(i);
        else if (res(i) > bounds.second(i))
            res(i) = bounds.second(i);
    }
    return res;
}

rw::math::Q PlannerUtil::clampPosition(
    const Device& device,
    const rw::math::Q& q)
{
    return clampPosition(device.getBounds(), q);
}
