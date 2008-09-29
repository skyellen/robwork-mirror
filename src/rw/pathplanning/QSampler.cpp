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

    class AbridgedSampler : public QSampler
    {
    public:
        AbridgedSampler(QSamplerPtr sampler, int cnt) :
            _sampler(sampler),
            _cnt(0),
            _maxCnt(cnt)
        {}

    private:
        Q doSample()
        {
            if (_cnt++ < _maxCnt)
                return _sampler->sample();
            else
                return Q();
        }

        bool doEmpty() const
        { return _cnt >= _maxCnt || _sampler->empty(); }

    private:
        QSamplerPtr _sampler;
        int _cnt;
        int _maxCnt;
    };

    class BoundsSampler : public QSampler
    {
    public:
        BoundsSampler(const Device::QBox& bounds) :
            _bounds(bounds)
        {}

    private:
        Q doSample()
        {
            return Math::ranQ(_bounds.first, _bounds.second);
        }

    private:
        Device::QBox _bounds;
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
            for (
                int cnt = 0;
                !_sampler->empty() && (_maxAttempts < 0 || cnt < _maxAttempts);
                ++cnt)
            {
                const Q q = _sampler->sample();
                if (!q.empty() && !_constraint->inCollision(q))
                    return q;
            }

            return Q();
        }

        bool doEmpty() const { return _sampler->empty(); }

    private:
        QSamplerPtr _sampler;
        QConstraintPtr _constraint;
        int _maxAttempts;
    };

    typedef QSamplerPtr T;
}

bool QSampler::empty() const { return doEmpty(); }

bool QSampler::doEmpty() const { return false; }

QSamplerPtr QSampler::makeEmpty()
{
    return ownedPtr(new EmptySampler());
}

QSamplerPtr QSampler::makeFixed(const Q& q)
{
    return ownedPtr(new FixedSampler(q));
}

QSamplerPtr QSampler::makeFinite(const std::vector<Q>& qs)
{
    return ownedPtr(new FiniteSampler(qs));
}

QSamplerPtr QSampler::makeFinite(QSamplerPtr sampler, int cnt)
{
    return ownedPtr(new AbridgedSampler(sampler, cnt));
}

QSamplerPtr QSampler::makeSingle(const Q& q)
{
    return makeFinite(std::vector<Q>(1, q));
}

QSamplerPtr QSampler::makeUniform(
    const Device::QBox& bounds)
{
    return ownedPtr(new BoundsSampler(bounds));
}

QSamplerPtr QSampler::makeUniform(
    const Device& device)
{
    return makeUniform(device.getBounds());
}

QSamplerPtr QSampler::makeUniform(
    DevicePtr device)
{
    return makeUniform(device->getBounds());
}

QSamplerPtr QSampler::makeNormalized(
    QSamplerPtr sampler,
    const QNormalizer& normalizer)
{
    return ownedPtr(new NormalizedSampler(sampler, normalizer));
}

QSamplerPtr QSampler::make(
    rw::common::Ptr<QIKSampler> sampler,
    const rw::math::Transform3D<>& target)
{
    return ownedPtr(new IKSampler(sampler, target));
}

QSamplerPtr QSampler::makeConstrained(
    QSamplerPtr sampler,
    QConstraintPtr constraint,
    int maxAttempts)
{
    return ownedPtr(new ConstrainedSampler(sampler, constraint, maxAttempts));
}
