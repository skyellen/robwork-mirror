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


#include "Math.hpp"
#include <rw/common/macros.hpp>

#include <boost/random.hpp>

#include <cassert>
#include <cmath>

using namespace rw::math;

namespace
{
    boost::mt19937 generator;
    boost::uniform_real<> distributor;

    typedef boost::normal_distribution<double> dist_type;

    boost::variate_generator<boost::mt19937, dist_type> normal_distribution(
        generator,
        boost::normal_distribution<double>(0, 1));
}

double Math::ranNormalDist(double mean, double sigma)
{
    return mean + sigma * normal_distribution();
}

double Math::ran()
{
	return distributor(generator);
}

void Math::seed(unsigned seed)
{
    // VC++ can't select the correct seed() method without a cast here.
    generator.seed(
        static_cast<boost::mt19937::result_type>(
            seed));
}

double Math::ran(double from, double to)
{
    RW_ASSERT(from <= to);

    return from + (to - from) * Math::ran();
}

int Math::ranI(int from, int to)
{
    return (int)floor(Math::ran(from, to));
}

Q Math::ranQ(const rw::math::Q& from, const rw::math::Q& to)
{
    RW_ASSERT(from.size() == to.size());

    const size_t len = from.size();
    Q result(len);
    for (size_t i = 0; i < len; i++)
        result(i) = ran(from(i), to(i));
    return result;
}

Q Math::ranQ(const std::pair<rw::math::Q, rw::math::Q>& bounds) {
	return ranQ(bounds.first, bounds.second);
}

Q Math::sqr(const Q& q)
{
    const size_t dim = q.size();
    Q result(dim);
    for (size_t i = 0; i < dim; i++) {
        result[i] = Math::sqr(q[i]);
    }
    return result;
}

Q Math::sqrt(const Q& q)
{
    const size_t dim = q.size();
    Q result(dim);
    for (size_t i = 0; i < dim; i++) {
        result[i] = std::sqrt(q[i]);
    }
    return result;
}

int Math::ceilLog2(const int n)
{
    RW_ASSERT(n > 0);
    int cnt = 0;
    int i = n;
    int a = 1;
    while (i != 1) {
        a <<= 1;
        i >>= 1;
        ++cnt;
    }
    if (a == n)
        return cnt;
    else
        return cnt + 1;
}
