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

#include "QExpand.hpp"
#include <rw/math/Math.hpp>
#include <rw/common/macros.hpp>

using namespace rw::pathplanning;
using namespace rw::math;

rw::math::Q QExpand::doSample()
{
    return expand(getSeed());
}

void QExpand::doSetSeed(const rw::math::Q& q)
{}

namespace
{
    Q expandUniform(
        const Q& q,
        const QExpand::QBox& outer,
        const QExpand::QBox& inner,
        double scale)
    {
        Q result(q.size());
        const int len = (int)q.size();
        for (int i = 0; i < len; i++) {
            const double lower =
                std::max(
                    outer.first[i],
                    q[i] + scale * inner.first[i]);

            const double upper =
                std::min(
                    outer.second[i],
                    q[i] + scale * inner.second[i]);

            if (lower <= upper) {
                result[i] = Math::ran(lower, upper);
            }
            else
                return Q();
        }

        return result;
    }

    class UniformBox : public QExpand
    {
    public:
        UniformBox(
            const QBox& outer,
            const QBox& inner)
            :
            _outer(outer),
            _inner(inner)
        {}

    private:
        rw::math::Q doExpand(const rw::math::Q& q)
        {
            return expandUniform(q, _outer, _inner, 1);
        }

    private:
        QBox _outer;
        QBox _inner;
    };

    class DecreasingUniformBox : public QExpand
    {
    public:
        DecreasingUniformBox(
            QConstraintPtr constraint,
            const QBox& outer,
            const QBox& inner)
            :
            _constraint(constraint),
            _outer(outer),
            _inner(inner)
        {}

    private:
        rw::math::Q doExpand(const rw::math::Q& q)
        {
            Q result(q.size());

            const int len = (int)q.size();
            for (double denom = 1;; ++denom) {
                const double scale = 1 / denom;
                const Q qn = expandUniform(q, _outer, _inner, scale);
                if (qn.empty() || !_constraint->inCollision(qn))
                    return qn;
            }
            return result;
        }

    private:
        QConstraintPtr _constraint;
        QBox _outer;
        QBox _inner;
    };

    QExpand::QBox makeInner(const QExpand::QBox& outer, double ratio)
    {
        const Q a = ratio * outer.first;
        const Q b = ratio * outer.second;
        const Q center = 0.5 * (a + b);
        return QExpand::QBox(a - center, b - center);
    }
}

std::auto_ptr<QExpand> QExpand::makeUniformBox(
    const QBox& outer,
    const QBox& inner)
{
    typedef std::auto_ptr<QExpand> T;
    return T(new UniformBox(outer, inner));
}

std::auto_ptr<QExpand> QExpand::makeUniformBox(
    const QBox& outer,
    double ratio)
{
    RW_ASSERT(ratio > 0);
    return makeUniformBox(outer, makeInner(outer, ratio));
}

std::auto_ptr<QExpand> QExpand::makeDecreasingUniformBox(
    QConstraintPtr constraint,
    const QBox& outer,
    const QBox& inner)
{
    typedef std::auto_ptr<QExpand> T;
    return T(new DecreasingUniformBox(constraint, outer, inner));
}

std::auto_ptr<QExpand> QExpand::makeDecreasingUniformBox(
    QConstraintPtr constraint,
    const QBox& outer,
    double ratio)
{
    return makeDecreasingUniformBox(constraint, outer, makeInner(outer, ratio));
}
