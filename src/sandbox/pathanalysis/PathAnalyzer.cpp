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

#include "PathAnalyzer.hpp"

#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/kinematics/FKRange.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rw::proximity;
using namespace rw::pathplanning;

PathAnalyzer::PathAnalyzer(DevicePtr device, const State& state):
    _device(device),
    _state(state)
{
}


PathAnalyzer::~PathAnalyzer()
{
}


PathAnalyzer::JointSpaceAnalysis PathAnalyzer::analyzeJointSpace(const QPath& path,
                                                                 QMetricPtr metric)
{
    JointSpaceAnalysis analysis;
    analysis.nodecount = path.size();

    EuclideanMetric<Q> euMetric;
    if (!metric)
        metric = &euMetric;

    analysis.length = Math::pathLength(path.begin(), path.end(), *metric);

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


PathAnalyzer::ClearanceAnalysis PathAnalyzer::analyzeClearance(const QPath& path, rw::proximity::DistanceCalculatorPtr distanceCalculator) {
    ClearanceAnalysis analysis;
    analysis.average = 0;
    analysis.min = 1e100;

    for (QPath::const_iterator it = path.begin()++; it != path.end()--; ++it) {
        _device->setQ(*it, _state);
        DistanceResult result = distanceCalculator->distance(_state);
        analysis.average += result.distance;
        analysis.min = std::min(analysis.min, result.distance);
    }
    analysis.average /= path.size();
    return analysis;
}



