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
