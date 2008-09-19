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
    if (a == n) return cnt;
    else return cnt + 1;
}
