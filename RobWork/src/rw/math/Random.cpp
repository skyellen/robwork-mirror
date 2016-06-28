/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "Random.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/TimerUtil.hpp>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>

using namespace rw::math;

namespace
{
    boost::mt19937 generator;
    boost::uniform_real<> distributor;

    typedef boost::normal_distribution<double> dist_type;

    boost::variate_generator<boost::mt19937&, dist_type> normal_distribution(
        generator,
        boost::normal_distribution<double>(0, 1));
}

Random::Random() {
}

Random::~Random() {
}

double Random::ranNormalDist(double mean, double sigma) {
    return mean + sigma * normal_distribution();
}

double Random::ran() {
	return distributor(generator);
}

void Random::seed(unsigned seed) {
    // VC++ can't select the correct seed() method without a cast here.
    generator.seed(
        static_cast<boost::mt19937::result_type>(
            seed));
}

void Random::seed() {
	seed((unsigned)rw::common::TimerUtil::currentTimeMs());
}

double Random::ran(double from, double to) {
    if(from>to) {
        RW_THROW("From must be smaller than to: " << from << ">" << to);
    } else if(from==to){
        return from;
    }

	double res = from;
	do {
		res = from + (to - from) * Random::ran();
	} while (res >= to);

	return res;

}

int Random::ranI(int from, int to)
{
    return (int)floor(Random::ran(from, to));
}
