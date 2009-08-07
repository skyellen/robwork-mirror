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


#include "TrajectoryFactory.hpp"
#include "TimedUtil.hpp"

#include <rw/math/Q.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <cfloat>

using namespace rw::trajectory;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::common;


namespace
{
    template <class X>
    Ptr<Trajectory<X> > makeLinearXTrajectory(const std::vector<Timed<X> >& path)
    {
        RW_ASSERT(path.empty() || path.size() >= 2);

        Ptr<InterpolatorTrajectory<X> > trajectory = ownedPtr(new InterpolatorTrajectory<X>);

        if (!path.empty()) {
            typedef typename std::vector<Timed<X> >::const_iterator I;
            I p = path.begin();
            I q = path.begin();
            for (++q; q != path.end(); ++p, ++q) {
                const double dt = q->getTime() - p->getTime();
                RW_ASSERT(dt >= 0);
                const X& a = p->getValue();
                const X& b = q->getValue();
                trajectory->add(new LinearInterpolator<X>(a, b, dt));
            }
        }
        return trajectory;
    }

    class FixedStateInterpolator : public Interpolator<State>
    {
    public:
        FixedStateInterpolator(const State& state) :
            _state(state),
            _zeroState(state)
        {
            for (size_t i = 0; i < state.size(); i++)
                _zeroState[i] = 0;
        }

        State x(double t) const { return _state; }
        State dx(double t) const { return _zeroState; }
        State ddx(double t) const { return _zeroState; }
        double duration() const { return DBL_MAX; }

    private:
        State _state;
        State _zeroState;
    };
}

StateTrajectoryPtr
TrajectoryFactory::makeFixedTrajectory(const State& state)
{
    Ptr<InterpolatorTrajectory<State> > trajectory = ownedPtr(new InterpolatorTrajectory<State>());
    trajectory->add(new FixedStateInterpolator(state));
    return trajectory;
}

StateTrajectoryPtr
TrajectoryFactory::makeLinearTrajectory(const TimedStatePath& path)
{
    return makeLinearXTrajectory(path);
}

StateTrajectoryPtr
TrajectoryFactory::makeLinearTrajectory(
    const StatePath& path,
    const WorkCell& workcell)
{
    return makeLinearTrajectory(
        TimedUtil::makeTimedStatePath(workcell, path));
}

StateTrajectoryPtr
TrajectoryFactory::makeLinearTrajectoryUnitStep(
    const StatePath& path)
{
    TimedStatePath timed;
    double time = 0;
    BOOST_FOREACH(const State& state, path) {
        timed.push_back(Timed<State>(time, state));
        ++time;
    }
    return makeLinearTrajectory(timed);
}

StateTrajectoryPtr TrajectoryFactory::makeEmptyStateTrajectory()
{
    return makeLinearTrajectory(TimedStatePath());
}

QTrajectoryPtr
TrajectoryFactory::makeLinearTrajectory(const TimedQPath& path)
{
    return makeLinearXTrajectory(path);
}

QTrajectoryPtr TrajectoryFactory::makeLinearTrajectory(const QPath& path, const Q& speed)
{
    return makeLinearTrajectory(TimedUtil::makeTimedQPath(speed, path));
}

QTrajectoryPtr TrajectoryFactory::makeLinearTrajectory(const QPath& path, const Device& device)
{
    return makeLinearTrajectory(path, device.getVelocityLimits());
}



QTrajectoryPtr TrajectoryFactory::makeLinearTrajectory(const QPath& path, QMetricPtr metric) {
	Ptr<InterpolatorTrajectory<Q> > trajectory = ownedPtr(new InterpolatorTrajectory<Q>());
	QPath::const_iterator it1 = path.begin();
	QPath::const_iterator it2 = path.begin();
	it2++;
	for (;it2 != path.end(); ++it2) {
		double d = metric->distance(*it1, *it2);
		Ptr<LinearInterpolator<Q> > interpolator = ownedPtr(new LinearInterpolator<Q>(*it1, *it2, d));
		trajectory->add(interpolator);
	}
	return trajectory;
}


QTrajectoryPtr TrajectoryFactory::makeEmptyQTrajectory()
{
    return makeLinearTrajectory(TimedQPath());
}
