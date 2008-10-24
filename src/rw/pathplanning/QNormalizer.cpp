/*********************************************************************
 * RobWork Version 0.3
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

#include "QNormalizer.hpp"

using namespace rw::math;
using namespace rw::pathplanning;

Q QNormalizer::fromNormalized(const Q& q) const
{
    Q r(q);
    setFromNormalized(r);
    return r;
}

Q QNormalizer::toNormalized(const Q& q) const
{
    Q r(q);
    setToNormalized(r);
    return r;
}

void QNormalizer::setFromNormalized(Q& q) const
{
    if (!_bounds.first.empty()) {
        for (size_t i = 0; i < q.size(); i++)
            q[i] =
                q[i] * (_bounds.second[i] - _bounds.first[i])
                + _bounds.first[i];
    }
}

void QNormalizer::setToNormalized(Q& q) const
{
    if (!_bounds.first.empty()) {
        for (size_t i = 0; i < q.size(); i++)
            q[i] =
                (q[i] - _bounds.first[i]) /
                (_bounds.second[i] - _bounds.first[i]);
    }
}
