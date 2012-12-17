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


BOOST_AUTO_TEST_CASE(Vector3DTest)
{
    BOOST_MESSAGE("- Testing VectorND");
    const VectorND<3,double> v1(1.0, 2.0, 3.0);
    const VectorND<3,double> v2(v1);
    const VectorND<3,double> v3 = v1 + v2;
    const VectorND<3, double> v4 = Vector3D<>(2.0, 4.0, 6.0);
    BOOST_CHECK(( v3 - v4 ).normInf() == 0);
    BOOST_CHECK( norm_inf(cross(v1, v2)) == 0);
	
	
    const VectorND<3, double> v5(3.0, 4.0, 5.0);
    const VectorND<3, double> v5_norm = normalize(v5);
    BOOST_CHECK(fabs(norm_2(v5_norm)-1)<1e-15);
    const double len = v5.norm2();
    BOOST_CHECK(fabs(v5_norm(0) - v5(0)/len)<1e-15);
    BOOST_CHECK(fabs(v5_norm(1) - v5(1)/len)<1e-15);
    
    VectorND<3, double> v6;
    v6(0) = len;
    BOOST_CHECK(v6(0) == len);

    const VectorND<4,std::string> vs1("x1", "y1", "z1", "a1");

    BOOST_CHECK(vs1[0] == "x1");
    BOOST_CHECK(vs1[1] == "y1");
    BOOST_CHECK(vs1[2] == "z1");
	BOOST_CHECK(vs1[3] == "a1");


	Eigen::Vector3d ev(1.1, 5.51, -10.3)
    const VectorND<double> vd(ev);
    const VectorND<int> vi = cast<int>(vd);
    BOOST_CHECK(vi(0) == 1);
    BOOST_CHECK(vi(1) == 5);
    BOOST_CHECK(vi(2) == -10);
}
