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


#include "QConstraint.hpp"
#include "QNormalizer.hpp"
#include "StateConstraint.hpp"

#include <rw/models/Models.hpp>
#include <rw/common/macros.hpp>
#include <rw/kinematics/State.hpp>
#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::pathplanning;

namespace
{
    class StateConstraintWrapper : public QConstraint
    {
    public:
        StateConstraintWrapper(StateConstraint::Ptr detector,
			Device::Ptr device,
            const State& state)
            :
            _detector(detector),
            _device(device),
            _state(state)
        {
            RW_ASSERT(device);
            RW_ASSERT(detector);
        }

    private:
        bool doInCollision(const Q& q) const
        {
            State state = _state;
            _device->setQ(q, state);
            return _detector->inCollision(state);
        }

		void doUpdate(const State& state) {
			_state = state;			
		}

		void doSetLog(Log::Ptr log) {
			_detector->setLog(log);
		}

    private:
		StateConstraint::Ptr _detector;
		Device::Ptr _device;
        State _state;
    };

    class MergedConstraints : public QConstraint
    {
    public:
        MergedConstraints(
			const std::vector<QConstraint::Ptr>& constraints) :
            _constraints(constraints)
        {}

    private:
        bool doInCollision(const Q& q) const
        {
			BOOST_FOREACH(const QConstraint::Ptr& sc, _constraints) {
                if (sc->inCollision(q))
                    return true;
            }
            return false;
        }

		void doUpdate(const rw::kinematics::State& state) {
			BOOST_FOREACH(const QConstraint::Ptr& sc, _constraints) {
				sc->update(state);
			}
		}

		void doSetLog(Log::Ptr log) {
			BOOST_FOREACH(const QConstraint::Ptr& sc, _constraints) {
				sc->setLog(log);
			}
		}

    private:
		std::vector<QConstraint::Ptr> _constraints;
    };

    class NormalizedConstraint : public QConstraint
    {
    public:
        NormalizedConstraint(
			QConstraint::Ptr constraint,
            const QNormalizer& normalizer)
            :
            _constraint(constraint),
            _normalizer(normalizer)
        {}

    private:
        bool doInCollision(const Q& raw_q) const
        {
            Q q = raw_q;
            _normalizer.setFromNormalized(q);
            return _constraint->inCollision(q);
        }

		void doSetLog(Log::Ptr log) {
			_constraint->setLog(log);
		}

    private:
		QConstraint::Ptr _constraint;
        QNormalizer _normalizer;
    };

    class FixedConstraint : public QConstraint
    {
    public:
        FixedConstraint(bool value) : _value(value) {}

    private:
        bool doInCollision(const Q&) const { 
			if (_log != NULL)		
				_log->debug()<<"FixedConstraint returns "<<_value;
			return _value; 			
		}

		void doSetLog(Log::Ptr log) {
			_log = log;
		}

    private:
        bool _value;
		Log::Ptr _log;
    };

    class BoundsConstraint : public QConstraint
    {
    public:
        BoundsConstraint(const Device::QBox& bounds)
            : _bounds(bounds) {}

    private:
        bool doInCollision(const Q& q) const
        {
			if (_log != NULL) {
				bool res = !Models::inBounds(q, _bounds);
				if (res) {
					_log->debug()<<"The configuration: "<<q<<" is outside bounds: Min="<<_bounds.first<<" Max="<<_bounds.second<<std::endl;
				}
				return res;
			} else 
				return !Models::inBounds(q, _bounds);
        }

		void doSetLog(Log::Ptr log) {
			_log = log;
		}

    private:
        Device::QBox _bounds;
		Log::Ptr _log;
    };
}

bool QConstraint::inCollision(const rw::math::Q& q) const
{
    return doInCollision(q);
}

void QConstraint::setLog(rw::common::Log::Ptr log) {
	doSetLog(log);
}

void QConstraint::update(const rw::kinematics::State& state) {
	doUpdate(state);
}

QConstraint::Ptr QConstraint::makeFixed(bool value)
{
    return ownedPtr(new FixedConstraint(value));
}

QConstraint::Ptr QConstraint::make(
	StateConstraint::Ptr detector,
	Device::Ptr device,
    const State& state)
{
    return ownedPtr(
        new StateConstraintWrapper(
            detector,
            device,
            state));
}

QConstraint::Ptr QConstraint::make(rw::common::Ptr<CollisionDetector> detector,
	Device::Ptr device,
    const State& state)
{
    return make(
        StateConstraint::make(detector),
        device,
        state);
}

QConstraint::Ptr QConstraint::makeMerged(
	const std::vector<QConstraint::Ptr>& constraints)
{
	return ownedPtr(new MergedConstraints(constraints));
}

QConstraint::Ptr QConstraint::makeMerged(const QConstraint::Ptr& ca,
								 	     const QConstraint::Ptr& cb)
{
	std::vector<QConstraint::Ptr> cs;
    cs.push_back(ca);
    cs.push_back(cb);
    return makeMerged(cs);
}

QConstraint::Ptr QConstraint::makeNormalized(
	const QConstraint::Ptr& constraint,
    const QNormalizer& normalizer)
{
    return ownedPtr(new NormalizedConstraint(constraint, normalizer));
}

QConstraint::Ptr QConstraint::makeNormalized(
	const QConstraint::Ptr& constraint,
    const std::pair<Q, Q>& bounds)
{
    return makeNormalized(constraint, QNormalizer(bounds));
}

QConstraint::Ptr QConstraint::makeNormalized(
	const QConstraint::Ptr& constraint,
    const Device& device)
{
    return makeNormalized(constraint, QNormalizer(device.getBounds()));
}

QConstraint::Ptr QConstraint::makeBounds(
    const Device::QBox& bounds)
{
    return ownedPtr(new BoundsConstraint(bounds));
}

