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
#include <rw/math/Vector2D.hpp>

using namespace rw::math;

BOOST_AUTO_TEST_CASE(Vector2DTest)
{
    BOOST_MESSAGE("- Testing Vector2D");
    Vector2D<> v1(1.0, 2.0);
    Vector2D<> v2(v1);
    Vector2D<> v3 = v1+v2;
    Vector2D<> v4 = Vector2D<>(2.0, 4.0);

    BOOST_CHECK ( (v3-v4).normInf() == 0);

    Vector2D<> v5(3.0, 4.0);
    Vector2D<> v5_norm = normalize(v5);
    BOOST_CHECK(fabs( v5_norm.norm2()-1)<1e-15);
    double l = v5.norm2();
    BOOST_CHECK(fabs(v5_norm(0)-v5(0)/l) < 1e-15);
    BOOST_CHECK(fabs(v5_norm(1)-v5(1)/l) < 1e-15);

    v5(0) = l;
    v5(1) = 2*l;
    BOOST_CHECK(v5(0) == l);
    BOOST_CHECK(v5(1) == 2*l);

    Vector2D<std::string> vs1("x1", "y1");

    BOOST_CHECK(vs1(0) == "x1");
    BOOST_CHECK(vs1(1) == "y1");

    double cr = cross(Vector2D<>(2,1), Vector2D<>(1, 3));
    BOOST_CHECK(cr == 5);

    Vector2D<double> vd(1.51, -2.51);
    Vector2D<int> vi = cast<int>(vd);
    BOOST_CHECK(vi(0) == 1);
    BOOST_CHECK(vi(1) == -2);

    Vector2D<double> vd1(0.4, -0.7);
    Vector2D<double> vd2(0.9, 0.1);
    BOOST_CHECK(fabs(dot(vd1,vd2) - 0.29) < 1e-15);

    /* Test comparison operators operator== and operator!= */
    const Vector2D<double> comp1(1.1, -2.2);
    const Vector2D<double> comp2(1.1, -2.2);
    BOOST_CHECK(comp1 == comp2);
    BOOST_CHECK(!(comp1 != comp2));
    const Vector2D<double> comp3(1.1, 2.2);
    BOOST_CHECK(comp1 != comp3);
    BOOST_CHECK(!(comp1 == comp3));
}
