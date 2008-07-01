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

#include "QSampler.hpp"

#include <rw/math/Math.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Models.hpp>
#include <boost/foreach.hpp>

using namespace rw::invkin;
using namespace rw::math;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::kinematics;

namespace
{
    Q randomQFromBox(const Q& lower, const Q& upper)
    {
        const int len = lower.size();
        Q q(len);
        for (int i = 0; i < len; i++) {
            q[i] = Math::ran(lower[i], upper[i]);
        }
        return q;
    }

    class EmptySampler : public QSampler
    {
    public:
        Q sample() { return Q(); }
        bool empty() const { return true; }
    };

    class FixedSampler : public QSampler
    {
    public:
        FixedSampler(const Q& q) : _q(q) {}

        Q sample() { return _q; }

    private:
        Q _q;
    };

    class BoundsSampler : public QSampler
    {
    public:
        BoundsSampler(const std::pair<Q, Q>& bounds) :
            _bounds(bounds)
        {}

        Q sample()
        {
            return randomQFromBox(_bounds.first, _bounds.second);
        }

    private:
        std::pair<Q, Q> _bounds;
    };

    class NormalizedSampler : public QSampler
    {
    public:
        NormalizedSampler(
            QSamplerPtr sampler,
            const QNormalizer& normalizer)
            :
            _sampler(sampler),
            _normalizer(normalizer)
        {}

        Q sample()
        {
            Q q = _sampler->sample();
            if (!q.empty()) _normalizer.setToNormalized(q);
            return q;
        }

    private:
        QSamplerPtr _sampler;
        QNormalizer _normalizer;
    };

    class IKSampler : public QSampler
    {
    public:
        IKSampler(
            IterativeIKPtr solver,
            DevicePtr device,
            QSamplerPtr seed,
            const State& state,
            const Transform3D<>& baseTend,
            int maxAttempts)
            :
            _solver(solver),
            _device(device),
            _seed(seed),
            _state(state),
            _baseTend(baseTend),
            _maxAttempts(maxAttempts)
        {}

        Q sample()
        {
            if (!_available.empty()) {
                const Q result = _available.back();
                _available.pop_back();
                return result;
            } else {
                Q result;
                for (int cnt = 0; cnt < _maxAttempts; ++cnt) {
                    const Q q = _seed->sample();
                    if (!q.empty()) {
                        _device->setQ(q, _state);

                        const std::vector<Q> qs =
                            _solver->solve(_baseTend, _state);

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
        IterativeIKPtr _solver;
        DevicePtr _device;
        QSamplerPtr _seed;
        State _state;
        Transform3D<> _baseTend;
        int _maxAttempts;
        std::vector<Q> _available;
    };

    class ConstrainedSampler : public QSampler
    {
    public:
        ConstrainedSampler(
            QSamplerPtr sampler,
            QConstraintPtr constraint,
            int maxAttempts)
            :
            _sampler(sampler),
            _constraint(constraint),
            _maxAttempts(maxAttempts)
        {}

        Q sample()
        {
            for (int cnt = 0; cnt < _maxAttempts; cnt++) {
                const Q q = _sampler->sample();
                if (!q.empty()) {
                    if (!_constraint->inCollision(q)) {
                        return q;
                    }
                }
            }
            return Q();
        }

    private:
        QSamplerPtr _sampler;
        QConstraintPtr _constraint;
        int _maxAttempts;
    };

    typedef std::auto_ptr<QSampler> T;
}

bool QSampler::empty() const { return false; }

std::auto_ptr<QSampler> QSampler::makeEmpty()
{
    return T(new EmptySampler());
}

std::auto_ptr<QSampler> QSampler::makeFixed(const Q& q)
{
    return T(new FixedSampler(q));
}

std::auto_ptr<QSampler> QSampler::makeUniform(
    const std::pair<Q, Q>& bounds)
{
    return T(new BoundsSampler(bounds));
}

std::auto_ptr<QSampler> QSampler::makeUniform(
    const Device& device)
{
    return makeUniform(device.getBounds());
}

std::auto_ptr<QSampler> QSampler::makeNormalized(
    QSamplerPtr sampler,
    const QNormalizer& normalizer)
{
    return T(new NormalizedSampler(sampler, normalizer));
}

std::auto_ptr<QSampler> QSampler::makeIterativeIK(
    IterativeIKPtr solver,
    DevicePtr device,
    const State& state,
    const Transform3D<>& baseTend,
    int maxAttempts)
{
    return T(
        new IKSampler(
            solver,
            device,
            // We can let the user provide this sampler also, if we like.
            QSampler::makeUniform(*device),
            state,
            baseTend,
            maxAttempts));
}

std::auto_ptr<QSampler> QSampler::makeConstrained(
    QSamplerPtr sampler,
    QConstraintPtr constraint,
    int maxAttempts)
{
    return T(new ConstrainedSampler(sampler, constraint, maxAttempts));
}
