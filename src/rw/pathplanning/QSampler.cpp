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
#include "QIKSampler.hpp"

#include <rw/math/Math.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/Models.hpp>
#include <boost/foreach.hpp>

using namespace rw::invkin;
using namespace rw::math;
using namespace rw::common;
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
    private:
        Q doSample() { return Q(); }
        bool doEmpty() const { return true; }
    };

    class FixedSampler : public QSampler
    {
    public:
        FixedSampler(const Q& q) : _q(q) {}

    private:
        Q doSample() { return _q; }

    private:
        Q _q;
    };

    class FiniteSampler : public QSampler
    {
    public:
        FiniteSampler(const std::vector<Q>& qs) :
            _qs(qs.rbegin(), qs.rend())
        {}

    private:
        Q doSample()
        {
            if (_qs.empty()) return Q();
            else {
                const Q result = _qs.back();
                _qs.pop_back();
                return result;
            }
        }

        bool doEmpty() const { return _qs.empty(); }

    private:
        std::vector<Q> _qs;
    };

    class BoundsSampler : public QSampler
    {
    public:
        BoundsSampler(const std::pair<Q, Q>& bounds) :
            _bounds(bounds)
        {}

    private:
        Q doSample()
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

    private:
        Q doSample()
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
            QIKSamplerPtr sampler,
            const Transform3D<>& target)
            :
            _sampler(sampler),
            _target(target)
        {}

    private:
        Q doSample()
        {
            return _sampler->sample(_target);
        }

        bool doEmpty() const { return _sampler->empty(); }

    private:
        QIKSamplerPtr _sampler;
        const Transform3D<> _target;
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

    private:
        Q doSample()
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

bool QSampler::empty() const { return doEmpty(); }

bool QSampler::doEmpty() const { return false; }

std::auto_ptr<QSampler> QSampler::makeEmpty()
{
    return T(new EmptySampler());
}

std::auto_ptr<QSampler> QSampler::makeFixed(const Q& q)
{
    return T(new FixedSampler(q));
}

std::auto_ptr<QSampler> QSampler::makeFinite(const std::vector<Q>& qs)
{
    return T(new FiniteSampler(qs));
}

std::auto_ptr<QSampler> QSampler::makeSingle(const Q& q)
{
    return makeFinite(std::vector<Q>(1, q));
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

std::auto_ptr<QSampler> QSampler::makeUniform(
    DevicePtr device)
{
    return makeUniform(device->getBounds());
}

std::auto_ptr<QSampler> QSampler::makeNormalized(
    QSamplerPtr sampler,
    const QNormalizer& normalizer)
{
    return T(new NormalizedSampler(sampler, normalizer));
}

std::auto_ptr<QSampler> QSampler::make(
    rw::common::Ptr<QIKSampler> sampler,
    const rw::math::Transform3D<>& target)
{
    return T(new IKSampler(sampler, target));
}

std::auto_ptr<QSampler> QSampler::makeConstrained(
    QSamplerPtr sampler,
    QConstraintPtr constraint,
    int maxAttempts)
{
    return T(new ConstrainedSampler(sampler, constraint, maxAttempts));
}
