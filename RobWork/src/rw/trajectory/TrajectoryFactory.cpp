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
#include <rw/trajectory/FixedInterpolator.hpp>
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
        RW_ASSERT(path.empty() || path.size() >= 1);

        Ptr<InterpolatorTrajectory<X> > trajectory = ownedPtr(new InterpolatorTrajectory<X>);

        if (path.size() == 1) {            
            return TrajectoryFactory::makeFixedTrajectory(path.front().getValue(), 0);
        }

        if (!path.empty()) {
            typedef typename std::vector<Timed<X> >::const_iterator I;
            I p = path.begin();
            I q = path.begin();
            for (++q; q != path.end(); ++p, ++q) {
                double dt = q->getTime() - p->getTime();
                //RW_ASSERT(dt >= 0);
                if(!(dt >= 0)){
                    RW_WARN("dt is wrong in trajectory. dt=" << dt << ". dt is force to 0.");
                    dt = 0;
                }
                const X& a = p->getValue();
                const X& b = q->getValue();
                trajectory->add(new LinearInterpolator<X>(a, b, dt));
            }
        }
        return trajectory;
    }

    /*template <class T>
    class FixedInterpolator: public Interpolator<T> 
    {
    public:
        FixedInterpolator(const T& value, double duration):
          _value(value),
          _zeroValue(value),
          _duration(duration)
        {
            for (size_t i = 0; i < _zeroValue.size(); i++)
                _zeroValue[i] = 0;
        }

        T x(double t) const { return _value; }
        T dx(double t) const { return _zeroValue; }
        T ddx(double t) const { return _zeroValue; }
        double duration() const { return _duration; }

    private:
        T _value;
        double _duration;
        T _zeroValue;

    };
*/
    /*class FixedStateInterpolator : public Interpolator<State>
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
    };*/
}

StateTrajectory::Ptr TrajectoryFactory::makeFixedTrajectory(const State& state, double duration)
{
    Ptr<InterpolatorTrajectory<State> > trajectory = ownedPtr(new InterpolatorTrajectory<State>());
    trajectory->add(ownedPtr(new FixedInterpolator<State>(state, duration)));
    return trajectory;
}

QTrajectory::Ptr TrajectoryFactory::makeFixedTrajectory(const rw::math::Q& q, double duration) {
    Ptr<InterpolatorTrajectory<Q> > trajectory = ownedPtr(new InterpolatorTrajectory<Q>());
    trajectory->add(ownedPtr(new FixedInterpolator<Q>(q, duration)));
    return trajectory;
}

StateTrajectory::Ptr TrajectoryFactory::makeLinearTrajectory(const TimedStatePath& path)
{
    return makeLinearXTrajectory(path);
}

StateTrajectory::Ptr TrajectoryFactory::makeLinearTrajectory(
    const StatePath& path,
    const WorkCell& workcell)
{
    return makeLinearTrajectory(
        TimedUtil::makeTimedStatePath(workcell, path));
}

StateTrajectory::Ptr TrajectoryFactory::makeLinearTrajectoryUnitStep(
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

StateTrajectory::Ptr TrajectoryFactory::makeEmptyStateTrajectory()
{
    return makeLinearTrajectory(TimedStatePath());
}

QTrajectory::Ptr TrajectoryFactory::makeLinearTrajectory(const TimedQPath& path)
{
    return makeLinearXTrajectory(path);
}

QTrajectory::Ptr TrajectoryFactory::makeLinearTrajectory(const QPath& path, const Q& speed)
{
    return makeLinearTrajectory(TimedUtil::makeTimedQPath(speed, path));
}

QTrajectory::Ptr TrajectoryFactory::makeLinearTrajectory(const QPath& path, const Device& device)
{
    return makeLinearTrajectory(path, device.getVelocityLimits());
}



QTrajectory::Ptr TrajectoryFactory::makeLinearTrajectory(const QPath& path, QMetric::Ptr metric) {
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


QTrajectory::Ptr TrajectoryFactory::makeEmptyQTrajectory()
{
    return makeLinearTrajectory(TimedQPath());
}

/**
 * @brief Constructs a linear trajectory for the path \b path. Times represents the
 * time for each segment
 *
 * @param path [in] path containing poses
 * @param times [in] times for each segment
 */
Transform3DTrajectory::Ptr TrajectoryFactory::makeLinearTrajectory(const Transform3DPath& path, const std::vector<double>& times) {
	InterpolatorTrajectory<Transform3D<> >::Ptr trajectory = ownedPtr(new InterpolatorTrajectory<Transform3D<> >());
	Transform3DPath::const_iterator it1 = path.begin();
	Transform3DPath::const_iterator it2 = path.begin();
	it2++;
	std::vector<double>::const_iterator it3 = times.begin();
	for (;it2 != path.end(); ++it2, ++it3) {		
		LinearInterpolator<Transform3D<> >::Ptr interpolator = ownedPtr(new LinearInterpolator<Transform3D<> >(*it1, *it2, *it3));
		trajectory->add(interpolator);
	}
	return trajectory;

}


Transform3DTrajectory::Ptr TrajectoryFactory::makeLinearTrajectory(const Transform3DPath& path, const rw::math::Transform3DMetric::Ptr metric) {
	InterpolatorTrajectory<Transform3D<> >::Ptr trajectory = ownedPtr(new InterpolatorTrajectory<Transform3D<> >());
	Transform3DPath::const_iterator it1 = path.begin();
	Transform3DPath::const_iterator it2 = path.begin();
	it2++;	
	for (;it2 != path.end(); ++it2) {		
		double duration = metric->distance(*it1, *it2);
		LinearInterpolator<Transform3D<> >::Ptr interpolator = ownedPtr(new LinearInterpolator<Transform3D<> >(*it1, *it2, duration));
		trajectory->add(interpolator);
	}
	return trajectory;

}
