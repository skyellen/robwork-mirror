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


#include "PathAnalyzer.hpp"

#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/models/Device.hpp>
#include <rw/proximity/DistanceCalculator.hpp>
#include <rw/kinematics/FKRange.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rw::proximity;
using namespace rw::pathplanning;

PathAnalyzer::PathAnalyzer(Device::Ptr device, const State& state):
    _device(device),
    _state(state)
{
}


PathAnalyzer::~PathAnalyzer()
{
}


PathAnalyzer::JointSpaceAnalysis PathAnalyzer::analyzeJointSpace(const QPath& path,
																 QMetric::Ptr metric)
{
    JointSpaceAnalysis analysis;
    analysis.nodecount = (double)path.size();

    EuclideanMetric<Q> euMetric;
    if (!metric)
        metric = &euMetric;

    analysis.length = pathLength(path.begin(), path.end(), *metric);

    return analysis;
}


PathAnalyzer::CartesianAnalysis PathAnalyzer::analyzeCartesian(const QPath& path, Frame* frame) {
    CartesianAnalysis analysis;
    if (path.size() < 1)
        return analysis;

    FKRange fkrange(_device->getBase(), frame, _state);
    _device->setQ(path.front(), _state);
    Transform3D<> preTransform = fkrange.get(_state);
    analysis.lower = preTransform.P();
    analysis.upper = preTransform.P();
    QPath::const_iterator it = path.begin();
    for (++it; it != path.end(); ++it) {
        _device->setQ(*it, _state);
        Transform3D<> transform = fkrange.get(_state);
        analysis.length += MetricUtil::dist2(preTransform.P(), transform.P());
        analysis.distances += Math::abs(preTransform.P() - transform.P());
        analysis.lower = Math::min(analysis.lower, transform.P());
        analysis.upper = Math::max(analysis.upper, transform.P());
        preTransform = transform;
    }
    return analysis;
}


PathAnalyzer::TimeAnalysis PathAnalyzer::analyzeTime(const QPath& path) {
    TimeAnalysis analysis;
    Q vellimits = _device->getVelocityLimits();

    QPath::const_iterator it1 = path.begin();
    QPath::const_iterator it2 = it1; it2++;
    for (; it2 != path.end(); ++it1, ++it2) {
        Q delta = (*it2) - (*it1);
        double maxtime = 0;
        for (size_t i = 0; i<delta.size(); i++) {
            maxtime = std::max(maxtime, delta(i)/vellimits(i));
        }
        analysis.time1 += maxtime;
    }

    //TODO Implement something to estimate the time when including acceleration limitations

    return analysis;
}


PathAnalyzer::ClearanceAnalysis PathAnalyzer::analyzeClearance(const QPath& path, rw::proximity::DistanceCalculator::Ptr distanceCalculator) {
    ClearanceAnalysis analysis;
    analysis.average = 0;
    analysis.min = 1e100;

    for (QPath::const_iterator it = path.begin()++; it != path.end()--; ++it) {
        _device->setQ(*it, _state);
        DistanceStrategy::Result result = distanceCalculator->distance(_state);
        analysis.average += result.distance;
        analysis.min = std::min(analysis.min, result.distance);
    }
    analysis.average /= path.size();
    return analysis;
}



