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


#include "QSampler.hpp"
#include "QNormalizer.hpp"
#include "QIKSampler.hpp"
#include "QConstraint.hpp"

#include <rw/math/Math.hpp>

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
		AbridgedSampler(QSampler::Ptr sampler, int cnt) :
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
		QSampler::Ptr _sampler;
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
			QSampler::Ptr sampler,
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
		QSampler::Ptr _sampler;
        QNormalizer _normalizer;
    };

    class IKSampler : public QSampler
    {
    public:
        IKSampler(
        	QIKSampler::Ptr sampler,
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
        QIKSampler::Ptr _sampler;
        const Transform3D<> _target;
    };

    class ConstrainedSampler : public QSampler
    {
    public:
        ConstrainedSampler(
			QSampler::Ptr sampler,
			QConstraint::CPtr constraint,
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
		QSampler::Ptr _sampler;
		QConstraint::CPtr _constraint;
        int _maxAttempts;
    };

//	typedef QSampler::Ptr T;
}

bool QSampler::empty() const { return doEmpty(); }

bool QSampler::doEmpty() const { return false; }

QSampler::Ptr QSampler::makeEmpty()
{
    return ownedPtr(new EmptySampler());
}

QSampler::Ptr QSampler::makeFixed(const Q& q)
{
    return ownedPtr(new FixedSampler(q));
}

QSampler::Ptr QSampler::makeFinite(const std::vector<Q>& qs)
{
    return ownedPtr(new FiniteSampler(qs));
}

QSampler::Ptr QSampler::makeFinite(QSampler::Ptr sampler, int cnt)
{
    return ownedPtr(new AbridgedSampler(sampler, cnt));
}

QSampler::Ptr QSampler::makeSingle(const Q& q)
{
    return makeFinite(std::vector<Q>(1, q));
}

QSampler::Ptr QSampler::makeUniform(
    const Device::QBox& bounds)
{
    return ownedPtr(new BoundsSampler(bounds));
}

QSampler::Ptr QSampler::makeUniform(
    const Device& device)
{
    return makeUniform(device.getBounds());
}

QSampler::Ptr QSampler::makeUniform(Device::CPtr device)
{
    return makeUniform(device->getBounds());
}

QSampler::Ptr QSampler::makeNormalized(QSampler::Ptr sampler,
    const QNormalizer& normalizer)
{
    return ownedPtr(new NormalizedSampler(sampler, normalizer));
}

QSampler::Ptr QSampler::make(QIKSampler::Ptr sampler,
    const rw::math::Transform3D<>& target)
{
    return ownedPtr(new IKSampler(sampler, target));
}

QSampler::Ptr QSampler::makeConstrained(QSampler::Ptr sampler,
									    QConstraint::CPtr constraint,
    int maxAttempts)
{
    return ownedPtr(new ConstrainedSampler(sampler, constraint, maxAttempts));
}

QSampler::Ptr QSampler::makeBoxDirectionSampler(
    const Device::QBox& bounds)
{
    const Q center = 0.5 * (bounds.first + bounds.second);
    const Device::QBox dirBounds(bounds.first - center, bounds.second - center);
    return QSampler::makeUniform(dirBounds);
}
