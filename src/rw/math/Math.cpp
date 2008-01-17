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
    
}

double Math::RanNormalDist(double mean, double sigma) {
    typedef boost::normal_distribution<double> dist_type; 
    dist_type norm_dist( mean, sigma ); 
    //Need to set up the random generator. Otherwise we seem to get the same number all the time
    Seed(RanI(-2147483647, 2147483647));
    boost::variate_generator< boost::mt19937, dist_type > rangen ( generator, 
                                                                 norm_dist ); 

    return rangen();
}

double Math::Ran()
{
	return distributor(generator);
}

void Math::Seed(unsigned seed)
{
    generator.seed(seed);
}

double Math::Ran(double from, double to)
{
    RW_ASSERT(from <= to);
    return from + (to - from) * Math::Ran();
}

int Math::RanI(int from, int to)
{
    return (int)floor(Math::Ran(from, to));
}
