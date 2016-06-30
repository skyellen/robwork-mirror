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


#include "Z3Planner.hpp"
#include "Z3QToQPlanner.hpp"

#include <rw/math/MetricUtil.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/pathplanning/QSampler.hpp>

using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rwlibs::pathplanners;

#include <boost/foreach.hpp>

namespace
{
    typedef QPath Path;

    /*
      A sampler of configurations that make up an orthonormal basis. The sampler
      becomes empty when the last base vector (or its inverse) has been returned.
    */
    class OrthogonalSampler : public QSampler
    {
    public:
		static QSampler::Ptr make(
            const Q& dir,
			QSampler::Ptr directionSampler,
			QMetric::Ptr metric,
            bool useBothWays)
        {
            return ownedPtr(
                new OrthogonalSampler(dir, directionSampler, metric, useBothWays));
        }

    private:
        OrthogonalSampler(const Q& dir,
			QSampler::Ptr directionSampler,
			QMetric::Ptr metric,
            bool useBothWays)
            :
            _directionSampler(directionSampler),
            _metric(metric),
            _useBothWays(useBothWays),
            _sampleCnt(0)
        {
            if (!dir.empty()) _base.push_back(dir);

            _dof = _directionSampler->sample().size();
        }

        Q doSample()
        {
            if (empty()) return Q();

            // If an even number of samples has been returned:
            if (!_useBothWays || _sampleCnt++ % 2 == 0) {

                while (true) {
                    Q dir = _directionSampler->sample();
                    if (dir.empty()) RW_THROW(
                        "Empty configuration returned by direction sampler.");

                    // Gram-Schmidt orthonormalization:
                    BOOST_FOREACH(const Q& e, _base) {
                        dir -= dot(dir, e) * e;
                    }

                    const double len = MetricUtil::norm2(dir);
                    if (len > 1e-10) {
                        _base.push_back(dir / len);
                        return _base.back();
                    }
                }
            } else {
                return -_base.back();
            }

            RW_ASSERT(0);
            return Q(); // To avoid a compiler warning.
        }

        bool doEmpty() const
        {
            return _base.size() == _dof && _sampleCnt % 2 == 0;
        }

    private:
		QSampler::Ptr _directionSampler;
		QMetric::Ptr _metric;
        bool _useBothWays;

        std::vector<Q> _base; // The orthonormal base so far.
        size_t _dof; // The number of degrees of freedom.
        int _sampleCnt; // The number of configurations returned.
    };

    class SlidingPlanner : public QToQPlanner
    {
    public:
        SlidingPlanner(
            const PlannerConstraint& constraint,
			QSampler::Ptr directionSampler,
			QConstraint::Ptr boundsConstraint,
			QMetric::Ptr metric,
            double extend,
            double slideImprovement)
            :
            _constraint(constraint),
            _directionSampler(directionSampler),
            _boundsConstraint(boundsConstraint),
            _metric(metric),
            _extend(extend),
            _slideImprovement(slideImprovement)
        {
            if (_slideImprovement < 0) _slideImprovement = 0.10 * _extend;

            RW_ASSERT(_extend >= 0);
            RW_ASSERT(_directionSampler);
            RW_ASSERT(_metric);
        }

    private:
        bool doQuery(const Q& start,
            const Q& goal,
            Path& result,
            const StopCriteria& stop)
        {
            // The defaults for how the sliding steps are selected:
            const bool useOnlyOrthogonal = true;
            const bool useBothWays = true;

            if (PlannerUtil::inCollision(_constraint, start) ||
                PlannerUtil::inCollision(_constraint, goal)) return false;

            Path path;
            const bool ok = findSurfacePoint(start, goal, path, stop);

            if (!ok) return false;

            // If we found a path right away:
            if (path.back() == goal) {
                RW_ASSERT(!PlannerUtil::inCollision(_constraint, path));
                result.insert(result.end(), path.begin(), path.end());
                return true;
            }

            while (!stop.stop()) {

                Q dir;
                if (useOnlyOrthogonal) {
                    const Q diff = goal - path.back();
                    const double len = MetricUtil::norm2(diff);

                    // If we are at the goal, but apparently couldn't get there,
                    // we return false here, and avoid a division by zero.
                    if (len < 1e-10) return false;
                    else dir = diff / len;
                }

				QSampler::Ptr baseSampler = OrthogonalSampler::make(
                    dir, _directionSampler, _metric, useBothWays);

                bool found = false;
                while (!found && !baseSampler->empty() && !stop.stop()) {
                    const Q dir = baseSampler->sample();
                    RW_ASSERT(!dir.empty());

                    const Q slide =
                        path.back() +
                        (_extend / _metric->distance(dir)) * dir;

                    if (!_boundsConstraint->inCollision(slide) &&
                        !PlannerUtil::inCollision(
                            _constraint, path.back(), slide, false, true))
                    {
                        Path slideToGoal;
                        const bool ok =
                            findSurfacePoint(slide, goal, slideToGoal, stop);
                        if (!ok) return false;

                        const Q& last = slideToGoal.back();

                        // A path to the goal was found:
                        if (last == goal) {
                            path.push_back(slide);
                            path.insert(
                                path.end(), slideToGoal.begin(), slideToGoal.end());

                            // This is a useful check to include if you update
                            // the algorithm:
                            // RW_ASSERT(!PlannerUtil::inCollision(_constraint, path));

                            result.insert(result.end(), path.begin(), path.end());
                            return true;
                        }

                        // A path to a configuration closer to the goal was found:
                        else if (
                            _metric->distance(path.back(), goal) -
                            _metric->distance(last, goal)
                            > _slideImprovement)
                        {
                            path.push_back(slide);
                            path.insert(
                                path.end(),
                                slideToGoal.begin(),
                                slideToGoal.end());

                            found = true;
                        } else {
                            // The step did not move use closer. Continue with
                            // another slide step.
                        }
                    }
                }

                // If no step was found, then no solution can be found.
                if (!found) return false;

                // Otherwise continue the sliding.
            }
            return false;
        }

        /*
          Making steps forward of size \b extend find the last configuration
          before a configuration is hit. The start configuration is assumed to
          be collision free and is always included in the result path. If a
          colliding configuration is never found, then \b goal will be the last
          element of \b result.
        */
        bool findSurfacePoint(
            const Q& start,
            const Q& goal,
            Path& path,
            const StopCriteria& stop) const
        {
            path.push_back(start);

            const Q diff = goal - start;
            const double len = _metric->distance(diff);

            const int steps = (int)(len / _extend);
            const Q step = steps > 0 ? (_extend / len) * diff : Q();

            bool collisionFound = false;
            for (int i = 0; !stop.stop() && i < steps; i++) {

                const Q next = path.back() + step;

                if (!PlannerUtil::inCollision(
                        _constraint, path.back(), next, false, true))
                {
                    path.push_back(next);
                } else {
                    collisionFound = true;
                    break;
                }
            }

            if (stop.stop()) return false;

            // If no collisions were found and the last step to the goal
            // configuration is also valid:
            if (!collisionFound &&
                !PlannerUtil::inCollision(
                    _constraint, path.back(), goal, false, false))
            {
                path.push_back(goal);
            }

            return true;
        }

    private:
        PlannerConstraint _constraint;
		QSampler::Ptr _directionSampler;
		QConstraint::Ptr _boundsConstraint;
		QMetric::Ptr _metric;
        double _extend;
        double _slideImprovement;
    };
}

QToQPlanner::Ptr Z3Planner::makeQToQPlanner(
	QSampler::Ptr sampler,
	QToQPlanner::Ptr localPlanner,
    int nodeCnt,
    int repeatCnt)
{
    return ownedPtr(
        new Z3QToQPlanner(sampler, localPlanner, nodeCnt, repeatCnt));
}

QToQPlanner::Ptr Z3Planner::makeQToQPlanner(const PlannerConstraint& constraint,
											Device::Ptr device)
{
    const int nodeCnt = 20;
    const int repeatCnt = -1;

    return makeQToQPlanner(
        QSampler::makeConstrained(
            QSampler::makeUniform(device),
            constraint.getQConstraintPtr()),
        makeSlidingQToQPlanner(constraint, device),
        nodeCnt,
        repeatCnt);
}

QToQPlanner::Ptr Z3Planner::makeSlidingQToQPlanner(const PlannerConstraint& constraint,
	QSampler::Ptr directionSampler,
	QConstraint::Ptr boundsConstraint,
	QMetric::Ptr metric,
    double extend,
    double slideImprovement)
{
    return ownedPtr(
        new SlidingPlanner(
            constraint,
            directionSampler,
            boundsConstraint,
            metric,
            extend,
            slideImprovement));
}

QToQPlanner::Ptr Z3Planner::makeSlidingQToQPlanner(const PlannerConstraint& constraint,
	Device::Ptr device,
	QMetric::Ptr metric,
    double extend,
    double slideImprovement)
{
    RW_ASSERT(device);

    const Device::QBox bounds = device->getBounds();

    if (!metric) {
        metric = PlannerUtil::normalizingInfinityMetric(bounds);

        if (extend >= 0) RW_THROW(
            "Metric is NULL, but extend distance is non-negative.");
        if (slideImprovement >= 0) RW_THROW(
            "Metric is NULL, but slide improvement distance is non-negative.");

        extend = 0.025;
    }

    return makeSlidingQToQPlanner(
        constraint,
        QSampler::makeBoxDirectionSampler(bounds),
        QConstraint::makeBounds(bounds),
        metric,
        extend,
        slideImprovement);
}
