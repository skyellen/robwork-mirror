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
#include "MetricFactory.hpp"
#include "Random.hpp"
#include <rw/common/macros.hpp>

#include <boost/math/special_functions/fpclassify.hpp> // boost::math::isnan()

#include <cmath>
#include <limits>

using namespace rw::math;


double Math::ranNormalDist(double mean, double sigma) {
	return Random::ranNormalDist(mean,sigma);
}

double Math::ran() {
	return Random::ran();
}

void Math::seed(unsigned seed) {
    Random::seed(seed);
}

void Math::seed() {
	Random::seed();
}

double Math::ran(double from, double to) {
    return Random::ran(from,to);
}

int Math::ranI(int from, int to) {
	return Random::ranI(from,to);
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

/**
 * @brief Returns a random direction in \b dim dimensions.
 *
 * The length of the vector is given by \b length;
 *
 * @param dim [in] Number of dimensions
 * @param length [in] Length of return vector. Default is 1;
 * @return Random direction
 */
Q Math::ranDir(size_t dim, double length) {
	Q q(dim);
	for (size_t i = 0; i<dim; i++) {
		q(i) = Math::ranNormalDist(0,1);
	}
	Metric<Q>::Ptr metric = MetricFactory::makeEuclidean<Q>();
	return length*q/metric->distance(q);
}

/**
 * @brief Returns a weighted random direction in \b dim dimensions.
 *
 * The length of the vector is given by \b length;
 *
 * @param dim [in] Number of dimensions
 * @param weights [in] Weights to use
 * @param length [in] Length of return vector when weights are applied as weighted Euclidean metric. Default is 1;
 * @return Random weigthed direction
 */
Q Math::ranWeightedDir(size_t dim, const rw::math::Q& weights, double length) {
	Q q(dim);
	for (size_t i = 0; i<dim; i++) {
		q(i) = Math::ranNormalDist(0,1);
	}
	Metric<Q>::Ptr metric = MetricFactory::makeWeightedEuclidean<Q>(weights);
	return length*q/metric->distance(q);
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

bool Math::isNaN(double d) {
	return boost::math::isnan(d);
}

double Math::NaN() {
    return std::numeric_limits<double>::quiet_NaN();
}
