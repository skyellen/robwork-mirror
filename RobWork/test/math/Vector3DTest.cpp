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
#include <rw/math/Vector3D.hpp>

using namespace rw::math;

namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return norm_inf(v.m());
    }

    double norm_2(const Vector3D<>& v)
    {
        return norm_2(v.m());
    }
}

BOOST_AUTO_TEST_CASE(Vector3DTest)
{
    BOOST_TEST_MESSAGE("- Testing Vector3D");
    const Vector3D<> v1(1.0, 2.0, 3.0);
    const Vector3D<> v2(v1);
    const Vector3D<> v3 = v1 + v2;
    const Vector3D<> v4 = Vector3D<>(2.0, 4.0, 6.0);
    BOOST_CHECK( norm_inf( v3 - v4 ) == 0);

    const Vector3D<> v5(3.0, 4.0, 5.0);
    const Vector3D<> v5_norm = normalize(v5);
    BOOST_CHECK(fabs(norm_2(v5_norm)-1)<1e-15);
    const double len = norm_2(v5);
    BOOST_CHECK(fabs(v5_norm(0) - v5(0)/len)<1e-15);
    BOOST_CHECK(fabs(v5_norm(1) - v5(1)/len)<1e-15);
    (fabs(v5_norm(2) - v5(2)/len)<1e-15);

    Vector3D<> v6;
    v6(0) = len;
    BOOST_CHECK(v6(0) == len);

    const Vector3D<std::string> vs1("x1", "y1", "z1");

    BOOST_CHECK(vs1[0] == "x1");
    BOOST_CHECK(vs1[1] == "y1");
    BOOST_CHECK(vs1[2] == "z1");

    BOOST_CHECK( norm_inf(cross(v1, v2)) == 0);

    const Vector3D<double> vd(1.1, 5.51, -10.3);
    const Vector3D<int> vi = cast<int>(vd);
    BOOST_CHECK(vi(0) == 1);
    BOOST_CHECK(vi(1) == 5);
    BOOST_CHECK(vi(2) == -10);

    /* Test comparison operators operator== and operator!= */
    const Vector3D<double> comp1(1.1, -2.2, 3.3);
    const Vector3D<double> comp2(1.1, -2.2, 3.3);
    BOOST_CHECK(comp1 == comp2);
    BOOST_CHECK(!(comp1 != comp2));
    const Vector3D<double> comp3(1.1, 2.2, -3.3);
    BOOST_CHECK(comp1 != comp3);
    BOOST_CHECK(!(comp1 == comp3));

}
