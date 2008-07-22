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
            Q result(q.size());

            const int len = (int)q.size();
            for (int i = 0; i < len; i++) {
                const double lower =
                    std::max(
                        _outer.first[i],
                        q[i] + _inner.first[i]);

                const double upper =
                    std::min(
                        _outer.second[i],
                        q[i] + _inner.second[i]);

                if (lower <= upper)
                    result[i] = Math::ran(lower, upper);
                else
                    return Q();
            }
            return result;
        }

    private:
        QBox _outer;
        QBox _inner;
    };
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

    const Q a = ratio * outer.first;
    const Q b = ratio * outer.second;
    const Q center = 0.5 * (a + b);
    const QBox inner(a - center, b - center);

    return makeUniformBox(outer, inner);
}
