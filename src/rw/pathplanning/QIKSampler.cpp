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

#include "QIKSampler.hpp"
#include <boost/foreach.hpp>

#include <rw/models/Models.hpp>

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
            DevicePtr device,
            const State& state,
            IterativeIKPtr solver,
            QSamplerPtr seed,
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

                for (
                    int cnt = 0;
                    cnt < _maxAttempts
                        && result.empty()
                        && !_seed->empty();
                    ++cnt)
                {
                    const Q q = _seed->sample();
                    if (!q.empty()) {
                        _device->setQ(q, _state);

                        const std::vector<Q> qs =
                            _solver->solve(target, _state);

                        BOOST_FOREACH(const Q& q, qs) {
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
        DevicePtr _device;
        State _state;
        IterativeIKPtr _solver;
        QSamplerPtr _seed;
        int _maxAttempts;
        std::vector<Q> _available;
    };

    class ConstrainedQIKSampler : public QIKSampler
    {
    public:
        ConstrainedQIKSampler(
            QIKSamplerPtr sampler,
            QConstraintPtr constraint,
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
        QIKSamplerPtr _sampler;
        QConstraintPtr _constraint;
        int _maxAttempts;
    };
}

bool QIKSampler::empty() const { return doEmpty(); }
bool QIKSampler::doEmpty() const { return false; }

QIKSamplerPtr QIKSampler::make(
    DevicePtr device,
    const State& state,
    IterativeIKPtr solver,
    QSamplerPtr seed,
    int maxAttempts)
{
    return ownedPtr(
        new IterativeQIKSampler(device, state, solver, seed, maxAttempts));
}

QIKSamplerPtr QIKSampler::makeConstrained(
    QIKSamplerPtr sampler,
    QConstraintPtr constraint,
    int maxAttempts)
{
    return ownedPtr(
        new ConstrainedQIKSampler(sampler, constraint, maxAttempts));
}
