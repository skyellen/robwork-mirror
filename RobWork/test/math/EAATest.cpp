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

#include <rw/math/EAA.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Constants.hpp>

using namespace rw::math;

namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return norm_inf(v.m());
    }
}

BOOST_AUTO_TEST_CASE( EAATest ){
    BOOST_TEST_MESSAGE("- Testing EAA");
    // 0 degree
    EAA<> e0(0.0, 0.0, 0.0);
    BOOST_CHECK(e0.angle() == 0);
    BOOST_CHECK(norm_inf(e0.axis()) == 0);
    EAA<> xe0( e0.toRotation3D() );
    BOOST_CHECK(xe0.angle() == e0.angle());
    BOOST_CHECK(norm_inf(xe0.axis() - e0.axis())==0);

    // 180 degree
    EAA<> e180_1(Pi, 0.0, 0.0);
    BOOST_CHECK(fabs(e180_1.angle()- Pi)<1e-16);
    BOOST_CHECK(norm_inf(e180_1.axis() - Vector3D<>(1.0, 0.0, 0.0)) == 0);
    EAA<> xe180_1( e180_1.toRotation3D() );
    BOOST_CHECK(fabs(xe180_1.angle() - e180_1.angle())<1e-16);
    BOOST_CHECK(norm_inf(xe180_1.axis() - e180_1.axis())==0);

    EAA<> e180_2(0.0, Pi, 0.0);
    BOOST_CHECK_CLOSE(e180_2.angle(), Pi, 1e-16);
    BOOST_CHECK_EQUAL(norm_inf(e180_2.axis() - Vector3D<>(0.0, 1.0, 0.0)), 0);
    EAA<> xe180_2(e180_2.toRotation3D());
    BOOST_CHECK_CLOSE(xe180_2.angle(), e180_2.angle(), 1e-16);
    BOOST_CHECK_EQUAL(norm_inf(xe180_2.axis() - e180_2.axis()), 0);

    EAA<> e180_3(0.0, 0.0, Pi);
    BOOST_CHECK_CLOSE(e180_3.angle(), Pi, 1e-16);
    BOOST_CHECK_EQUAL(norm_inf(e180_3.axis() - Vector3D<>(0.0, 0.0, 1.0)), 0);
    EAA<> xe180_3(e180_3.toRotation3D());
    BOOST_CHECK_CLOSE(xe180_3.angle() , e180_3.angle(),1e-16);
    BOOST_CHECK_EQUAL(norm_inf(xe180_3.axis() - e180_3.axis()),0);

    BOOST_TEST_MESSAGE("-- Testing different sign combinations for 180 degree rotations");
    double val1 = 0.3;
    double val2 = 0.4;
    double val3 = 0.5;
    for (int sign1 = -1; sign1 <= 1; sign1++) {
        for (int sign2 = -1; sign2 <= 1; sign2++) {
            for (int sign3 = -1; sign3 <= 1; sign3++) {
                if (sign1 == 0 && sign2 == 0 && sign3 == 0)
                	continue; // the zero case is not tested here
            	Vector3D<> axisInput(sign1*val1, sign2*val2, sign3*val3);
                BOOST_TEST_MESSAGE("--- Sign combination: " << axisInput);
            	axisInput = normalize(axisInput);
                EAA<> e180(axisInput*Pi);
                BOOST_CHECK_CLOSE(e180.angle(), Pi, 1e-13);
                Vector3D<> e180axis = e180.axis();
                BOOST_CHECK_SMALL(norm_inf(e180axis - axisInput), 1e-15);
                EAA<> xe180(e180.toRotation3D());
                BOOST_CHECK_CLOSE(xe180.angle() , e180.angle(),1e-13);
                Vector3D<> xe180axis = xe180.axis();
                // vector can point both ways and still be valid
                if ((xe180axis - e180axis).norm2() > (-xe180axis - e180axis).norm2())
                	xe180axis = -xe180axis;
                BOOST_CHECK_SMALL(norm_inf(xe180axis - e180axis),1e-15);
            }
        }
    }

    // 90 degree's around x axis
    Vector3D<> v1(1.0, 0.0, 0.0);
    EAA<> e1(v1, Pi / 2.0);
    BOOST_CHECK_CLOSE(e1.angle() , Pi / 2.0, 1e-16);
    BOOST_CHECK_EQUAL(norm_inf( e1.axis() - v1 ) , 0);
    Rotation3D<> rot = e1.toRotation3D();
    EAA<> e2(rot);
    BOOST_CHECK_EQUAL(norm_inf( e1.axis() - e2.axis() ), 0);
    BOOST_CHECK_CLOSE( e1.angle() , e2.angle(), 1e-16 );

    /* Test comparison operators operator== and operator!= */
    const EAA<double> comp1(1.1, -2.2, 3.3);
    const EAA<double> comp2(1.1, -2.2, 3.3);
    BOOST_CHECK(comp1 == comp2);
    BOOST_CHECK(!(comp1 != comp2));
    const EAA<double> comp3(1.1, 2.2, -3.3);
    BOOST_CHECK(comp1 != comp3);
    BOOST_CHECK(!(comp1 == comp3));
    
}
