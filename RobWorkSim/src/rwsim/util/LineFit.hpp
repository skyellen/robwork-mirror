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

#ifndef RWSIM_UTIL_LINEFIT_HPP
#define RWSIM_UTIL_LINEFIT_HPP

#include "LinePolar.hpp"
#include "P2D.hpp"
#include <vector>

namespace LineFit
{
    typedef std::vector<P2D>::const_iterator const_iterator;
    typedef std::pair<const_iterator, const_iterator> const_iterator_pair;

    LinePolar fit(const_iterator_pair range);
    LinePolar fit(const_iterator a, const_iterator b);
    LinePolar fit(const std::vector<P2D>& pnts);
}

#endif
