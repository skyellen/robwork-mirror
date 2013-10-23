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
#include <rw/math/VectorND.hpp>

using namespace rw::math;


BOOST_AUTO_TEST_CASE(VectorNDTest)
{
    BOOST_MESSAGE("- Testing VectorND");
    const VectorND<3,double> v1 = VectorND<3,double>::zero();
    const VectorND<3,double> v2 = VectorND<3,double>::zero();
    BOOST_CHECK(( v1 - v2 ).normInf() == 0);
	
    VectorND<3, double> v6;
    v6(0) = 0.3;
    BOOST_CHECK(v6(0) == 0.3);

	Eigen::Vector3d ev(1.1, 5.51, -10.3);
    const VectorND<3,double> vd(ev);


    std::vector< VectorND<6,double> > vOfVectors;

    vOfVectors.push_back( VectorND<6,double>::zero() );
    vOfVectors.push_back( VectorND<6,double>::zero() );
    vOfVectors.push_back( VectorND<6,double>::zero() );
    vOfVectors.push_back( VectorND<6,double>::zero() );

}
