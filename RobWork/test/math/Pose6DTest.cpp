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


#include "../TestSuiteConfig.hpp"
#include <rw/math/Pose6D.hpp>

using namespace rw::math;

BOOST_AUTO_TEST_CASE(Pose6DTest){
	BOOST_TEST_MESSAGE("- Testing Pose6D");
    Pose6D<double> p(1.1,2.2,3.3,4.4,5.5,6.6);

    BOOST_CHECK(p(0) == 1.1);
    BOOST_CHECK(p(1) == 2.2);
    BOOST_CHECK(p(2) == 3.3);
    BOOST_CHECK_CLOSE(p(3),4.4,1e-15);
    BOOST_CHECK_CLOSE(p(4),5.5,1e-15);
    BOOST_CHECK_CLOSE(p(5),6.6,1e-15);

	Pose6D<float> pf;
	pf = cast<float>(p);
    for (size_t i = 0; i<6; i++)
        BOOST_CHECK_CLOSE(pf(i), (float)p(i), 1e-15f);
	pf = rw::math::cast<float>(p); // qualified lookup
	for (size_t i = 0; i<6; i++)
		BOOST_CHECK_CLOSE(pf(i), (float)p(i), 1e-15f);
}
