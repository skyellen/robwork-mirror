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


#include "QIKSampler.hpp"
#include "QConstraint.hpp"
#include "QSampler.hpp"
#include <boost/foreach.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/Models.hpp>
#include <rw/invkin/IterativeIK.hpp>

using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::invkin;
using namespace rw::common;

namespace
{
    class IterativeQIKSampler : public QIKSampler
    {
    public:
        IterativeQIKSampler(
			Device::Ptr device,
            const State& state,
			IterativeIK::Ptr solver,
			QSampler::Ptr seed,
            int maxAttempts)
            :
            _device(device),
            _state(state),
            _solver(solver),
            _seed(seed),
            _maxAttempts(maxAttempts)
        {
            if (!_solver) _solver = IterativeIK::makeDefault(device, state);
            if (!_seed) _seed = QSampler::makeUniform(device);
            if (_maxAttempts < 0) _maxAttempts = 15;
        }

    private:
        Q doSample(const Transform3D<>& target)
        {
            if (!_available.empty()) {
                const Q result = _available.back();
                _available.pop_back();
                return result;
            } else {
                Q result;

                for (int cnt = 0; cnt < _maxAttempts && result.empty() && !_seed->empty(); ++cnt) {
                    const Q q = _seed->sample();
                    if (!q.empty()) {
                        _device->setQ(q, _state);

                        const std::vector<Q> qs = _solver->solve(target, _state);

                        BOOST_FOREACH(const Q& q, qs){
                        if (Models::inBounds(q, *_device)) {
                            if (result.empty())
                            result = q;
                            else {
                                // Save this solution for later.
                                _available.push_back(q);
                            }
                        }
                    }
                }
            }
            return result;
        }
    }

    private:
		Device::Ptr _device;
        State _state;
		IterativeIK::Ptr _solver;
		QSampler::Ptr _seed;
        int _maxAttempts;
        std::vector<Q> _available;
    };

    class ConstrainedQIKSampler : public QIKSampler
    {
    public:
        ConstrainedQIKSampler(
			QIKSampler::Ptr sampler,
			QConstraint::Ptr constraint,
            int maxAttempts)
            :
            _sampler(sampler),
            _constraint(constraint),
            _maxAttempts(maxAttempts)
        {}

    private:
        Q doSample(const Transform3D<>& target)
        {
            for (
                int cnt = 0;
                !_sampler->empty() && (_maxAttempts < 0 || cnt < _maxAttempts);
                ++cnt)
            {
                const Q q = _sampler->sample(target);
                if (!q.empty() && !_constraint->inCollision(q))
                    return q;
            }

            return Q();
        }

        bool doEmpty() const { return _sampler->empty(); }

    private:
		QIKSampler::Ptr _sampler;
		QConstraint::Ptr _constraint;
        int _maxAttempts;
    };
}

bool QIKSampler::empty() const { return doEmpty(); }
bool QIKSampler::doEmpty() const { return false; }

QIKSampler::Ptr QIKSampler::make(Device::Ptr device,
    const State& state,
	IterativeIK::Ptr solver,
	QSampler::Ptr seed,
    int maxAttempts)
{
    return ownedPtr(
        new IterativeQIKSampler(device, state, solver, seed, maxAttempts));
}

QIKSampler::Ptr QIKSampler::makeConstrained(
	QIKSampler::Ptr sampler,
	QConstraint::Ptr constraint,
    int maxAttempts)
{
    return ownedPtr(
        new ConstrainedQIKSampler(sampler, constraint, maxAttempts));
}
