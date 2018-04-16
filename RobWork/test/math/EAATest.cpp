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
#include <rw/math/Transform3D.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>

BOOST_AUTO_TEST_CASE(EAATest_ADL) {
	rw::math::Vector3D<> vec1(0.1, 0.2, 0.3);
	rw::math::EAA<> eaa2(0.4, 0.5, 0.6);
	rw::math::Vector3D<> c;

	c = cross(vec1, eaa2);
	BOOST_CHECK(fabs(c[0] - (0.2*0.6 - 0.3*0.5)) < 1e-15);
	BOOST_CHECK(fabs(c[1] - (0.1*0.6 - 0.3*0.4)) < 1e-15);
	BOOST_CHECK(fabs(c[2] - (0.1*0.5 - 0.4*0.2)) < 1e-15);
}

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

	// Test casting
	{
		EAA<> eaacast(0.1, 0.2, 0.3);
		EAA<float> eaaf;
		eaaf = cast<float>(eaacast);
		for (size_t i = 0; i < 3; i++)
			BOOST_CHECK(eaaf(i) == (float)eaacast(i));
		eaaf = rw::math::cast<float>(eaacast); // qualified lookup
		for (size_t i = 0; i < 3; i++)
			BOOST_CHECK(eaaf(i) == (float)eaacast(i));
	}

	// Test cross product
	{
		Vector3D<> vec1(0.1, 0.2, 0.3);
		EAA<> eaa2(0.4, 0.5, 0.6);
		Vector3D<> c;

		c = cross(vec1, eaa2);
		BOOST_CHECK(fabs(c[0] - (0.2*0.6 - 0.3*0.5)) < 1e-15);
		BOOST_CHECK(fabs(c[1] - (0.1*0.6 - 0.3*0.4)) < 1e-15);
		BOOST_CHECK(fabs(c[2] - (0.1*0.5 - 0.4*0.2)) < 1e-15);
		c = rw::math::cross(vec1, eaa2); // qualified lookup
		BOOST_CHECK(fabs(c[0] - (0.2*0.6 - 0.3*0.5)) < 1e-15);
		BOOST_CHECK(fabs(c[1] - (0.1*0.6 - 0.3*0.4)) < 1e-15);
		BOOST_CHECK(fabs(c[2] - (0.1*0.5 - 0.4*0.2)) < 1e-15);
	}

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
    const double val1 = 0.3;
    const double val2 = 0.4;
    const double val3 = 0.5;
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
    BOOST_TEST_MESSAGE("-- Testing different sign combinations for 180 degree rotations - epsilon");
    const double eps = 1e-7; // should be less than the hardcoded threshold in EAA.cpp
    for (int sign1 = -1; sign1 <= 1; sign1++) {
    	for (int sign2 = -1; sign2 <= 1; sign2++) {
    		for (int sign3 = -1; sign3 <= 1; sign3++) {
    			if (sign1 == 0 && sign2 == 0 && sign3 == 0)
    				continue; // the zero case is not tested here
    			Vector3D<> axisInput(sign1*val1, sign2*val2, sign3*val3);
    			BOOST_TEST_MESSAGE("--- Sign combination: " << axisInput);
    			axisInput = normalize(axisInput);
    			EAA<> e180(axisInput*(Pi-eps));
    			BOOST_CHECK_CLOSE(e180.angle(), Pi-eps, 1e-13);
    			Vector3D<> e180axis = e180.axis();
    			BOOST_CHECK_SMALL(norm_inf(e180axis - axisInput), 1e-15);
    			EAA<> xe180(e180.toRotation3D());
    			BOOST_CHECK_CLOSE(xe180.angle(), e180.angle(), 5e-13);
    			Vector3D<> xe180axis = xe180.axis();
    			BOOST_CHECK_SMALL(norm_inf(xe180axis - e180axis),1e-7);
    		}
    	}
    }
    BOOST_TEST_MESSAGE("-- Testing different sign combinations for 180 degree rotations + epsilon");
    for (int sign1 = -1; sign1 <= 1; sign1++) {
    	for (int sign2 = -1; sign2 <= 1; sign2++) {
    		for (int sign3 = -1; sign3 <= 1; sign3++) {
    			if (sign1 == 0 && sign2 == 0 && sign3 == 0)
    				continue; // the zero case is not tested here
    			Vector3D<> axisInput(sign1*val1, sign2*val2, sign3*val3);
    			BOOST_TEST_MESSAGE("--- Sign combination: " << axisInput);
    			axisInput = normalize(axisInput);
    			EAA<> e180(axisInput*(Pi+eps));
    			BOOST_CHECK_CLOSE(e180.angle(), Pi+eps, 1e-13);
    			Vector3D<> e180axis = e180.axis();
    			BOOST_CHECK_SMALL(norm_inf(e180axis - axisInput), 1e-15);
    			EAA<> xe180(e180.toRotation3D());
    			BOOST_CHECK_CLOSE(xe180.angle(), Pi-eps, 5e-13); // should choose angle < Pi
    			Vector3D<> xe180axis = xe180.axis();
    			BOOST_CHECK_SMALL(norm_inf(xe180axis + e180axis),1e-7); // should flip vector to get angle < Pi
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
    

	/** Test different "problematic setups which previously has resulted in NaN errors */

	Transform3D<>t0(Vector3D<>(0.0004406670, 0.1325120000, 0.0236778000), Rotation3D<>(0.9990961814, -0.0003891806, -0.0425144844, 0.0004163897, 0.9999992493, 0.0006269312, 0.0425143000, -0.0006440670, 0.9990960000));
	Transform3D<>t1(Vector3D<>(0.0004406670, 0.1325120000, 0.0236778000), Rotation3D<>(-0.0000000000, -0.0000000000, 1.0000000000, 0.0000000000, -1.0000000000, -0.0000000000, 1.0000000000, 0.0000000000, 0.0000000000));
	Rotation3D<>r0((inverse(t0)*t1).R());
	r0.normalize();
	EAA<> eaa0(r0);
	BOOST_CHECK(Math::isNaN(eaa0.angle()) == false);
	


	Transform3D<>t2(Vector3D<>(0.0004406670, 0.1325120000, 0.0236778000), Rotation3D<>(-1.0000000000, -0.0000000000, 0.0000000000, 0.0000000000, -1.0000000000, 0.0000000000, -0.0000000000, 0.0000000000, 1.0000000000));
	Rotation3D<>r1((inverse(t0)*t2).R());
	r1.normalize();
	EAA<> eaa1(r1);
	BOOST_CHECK(Math::isNaN(eaa1.angle()) == false);

	Transform3D<>t3(Vector3D<>(0.0004406670, 0.1325120000, 0.0236778000), Rotation3D<>(1.0000000000, -0.0000000000, -0.0000000000, -0.0000000000, -1.0000000000, 0.0000000000, -0.0000000000, -0.0000000000, -1.0000000000));
	Rotation3D<>r2((inverse(t0)*t3).R());
	r2.normalize();
	EAA<> eaa2(r2);
	BOOST_CHECK(Math::isNaN(eaa2.angle()) == false);

	Transform3D<>t4(Vector3D<>(0.0004406670, 0.1325120000, 0.0236778000), Rotation3D<>(-0.9993793864, -0.0352195084, -0.0004122380, -0.0004294999, 0.0004964944, 0.9999993429, -0.0352193000, 0.9993790000, -0.0005113220));
	Transform3D<>t5(Vector3D<>(0.0004406670, 0.1325120000, 0.0236778000), Rotation3D<>(0.0000000000, -1.0000000000, -0.0000000000, 0.0000000000, 0.0000000000, -1.0000000000, 1.0000000000, 0.0000000000, 0.0000000000));
	Rotation3D<>r3((inverse(t4)*t5).R());
	r3.normalize();
	EAA<> eaa3(r3);
	BOOST_CHECK(Math::isNaN(eaa3.angle()) == false);


	Transform3D<>t6(Vector3D<>(0.0004406670, 0.1325120000, 0.0236778000), Rotation3D<>(0.0000000000, 1.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, -1.0000000000, -1.0000000000, 0.0000000000, 0.0000000000));
	Rotation3D<>r4((inverse(t4)*t6).R());
	r4.normalize();
	EAA<> eaa4(r4);
	BOOST_CHECK(Math::isNaN(eaa4.angle()) == false);

	Transform3D<>t7(Vector3D<>(0.0004406670, 0.1325120000, 0.0236778000), Rotation3D<>(-1.0000000000, 0.0000000000, 0.0000000000, -0.0000000000, 0.0000000000, -1.0000000000, -0.0000000000, -1.0000000000, -0.0000000000));
	Rotation3D<>r5((inverse(t4)*t7).R());
	r5.normalize();
	EAA<> eaa5(r5);
	BOOST_CHECK(Math::isNaN(eaa5.angle()) == false);


}
