#include "PathAnalyzer.hpp"

#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/kinematics/FKRange.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;

PathAnalyzer::PathAnalyzer(Device* device, const State& state):
    _device(device),
    _state(state)
{
}

PathAnalyzer::~PathAnalyzer()
{
}


PathAnalyzer::JointSpaceAnalysis PathAnalyzer::analyzeJointSpace(Path& path, Metric<double>* metric) {
    JointSpaceAnalysis analysis;
    analysis.nodecount = path.size();

    EuclideanMetric<double> euMetric;
    if (metric == NULL)
        metric = &euMetric;

    analysis.length = 0;
    Path::const_iterator it1 = path.begin();
    Path::const_iterator it2 = it1; it2++;
    for (; it2 != path.end(); ++it1, ++it2) {
        analysis.length += metric->distance(*it1, *it2);
    }

    return analysis;
}

PathAnalyzer::CartesianAnalysis PathAnalyzer::analyzeCartesian(rw::pathplanning::Path& path, Frame* frame) {
    CartesianAnalysis analysis;
    if (path.size() < 1)
        return analysis;

    FKRange fkrange(_device->getBase(), frame, _state);
    _device->setQ(path.front(), _state);
    Transform3D<> preTransform = fkrange.get(_state);
    analysis.lower = preTransform.P();
    analysis.upper = preTransform.P();
    Path::const_iterator it = path.begin();
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


PathAnalyzer::TimeAnalysis PathAnalyzer::analyzeTime(rw::pathplanning::Path& path) {
    TimeAnalysis analysis;
    Q vellimits = _device->getVelocityLimits();

    Path::const_iterator it1 = path.begin();
    Path::const_iterator it2 = it1; it2++;
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


PathAnalyzer::ClearanceAnalysis PathAnalyzer::analyzeClearance(Path& path, rw::proximity::DistanceCalculator* distanceCalculator) {
    ClearanceAnalysis analysis;
    analysis.average = 0;
    analysis.min = 1e100;

    for (Path::const_iterator it = path.begin()++; it != path.end()--; ++it) {
        _device->setQ(*it, _state);
        DistanceResult result = distanceCalculator->distance(_state);
        analysis.average += result.distance;
        analysis.min = std::min(analysis.min, result.distance);
    }
    analysis.average /= path.size();
    return analysis;
}



