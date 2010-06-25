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


#include <rw/math/Quaternion.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>

#include <iostream>

using namespace rw::math;

#include <boost/test/unit_test.hpp>

typedef Quaternion<double> QuatD;	// just abbreviations
typedef Quaternion<float>  QuatF;

bool close_enough(Quaternion<> q1,Quaternion<> q2){
    return fabs((q1).getQx()-(q2).getQx())<1e-16 &&
    fabs((q1).getQy()-(q2).getQy())<1e-16 &&
    fabs((q1).getQz()-(q2).getQz())<1e-16 &&
    fabs((q1).getQw()-(q2).getQw())<1e-16;
}

BOOST_AUTO_TEST_CASE(QuaternionTest){
    BOOST_MESSAGE("- Testing Quaternion");

    Rotation3D<> r3d(-0.99999669678603531, -0.0020716226013274717, -0.001521445633434071,
    				 -0.001521761408544605, 0.00015086113440790838, 0.99999883072019702,
    				 0.0018535970374102858, 0.99999827110886175, -0.00014804031424759767);
    Vector3D<> v3d(-0.00046301776312795973,-0.63014974943015156,0.09990255421621469);
    Transform3D<> t3d(v3d,r3d);

    Quaternion<> q(r3d);
    std::cout << "before \n" << r3d << std::endl;
    std::cout << "before \n" << q << std::endl;
    Rotation3D<> r3d_after = q.toRotation3D();
    std::cout << "after \n" << r3d_after << std::endl;

    std::cout << "after \n" << Quaternion<>(r3d_after) << std::endl;
    std::cout << "RPY bfore \n" << RPY<>(r3d) << std::endl;
    std::cout << "RPY after \n" << RPY<>(r3d_after) << std::endl<<std::endl;

    //Test Quaternion(T a, T b, T c, T d) constructor
    Quaternion<> q1(1.0,2.0,3.0,4.0);
    BOOST_CHECK(q1.getQx()==1.0);
    BOOST_CHECK(q1.getQy()==2.0);
    BOOST_CHECK(q1.getQz()==3.0);
    BOOST_CHECK(q1.getQw()==4.0);

    //Test toRotation3D and Quaternion(const Rotation3D&) constructor
    Rotation3D<> r1 = q1.toRotation3D();
    Quaternion<> q2(r1);
    close_enough(q1,q2);

    // Test arithmetic functions
    Quaternion<> a(1,2,3,4), b(4,3,2,1), c(0,0,0,0), h(0,0,0,0);
    BOOST_CHECK(close_enough( a,         Quaternion<>(1.0,2.0,3.0,4.0) )); // quat - quat
    BOOST_CHECK(close_enough( +a,        Quaternion<>(1.0,2.0,3.0,4.0) ));
    BOOST_CHECK(close_enough( -a,        Quaternion<>(-1.0,-2.0,-3.0,-4.0) ));

    BOOST_CHECK(close_enough( (a + b),     Quaternion<>(5.0,5.0,5.0,5.0) ));
    BOOST_CHECK(close_enough( (a += b),    Quaternion<>(5,5,5,5) ));
    BOOST_CHECK(close_enough( (a - b),     Quaternion<>(1,2,3,4) ));
    BOOST_CHECK(close_enough( (a -= b),    Quaternion<>(1,2,3,4) ));

    h = a;
    BOOST_CHECK(close_enough( h *= b,    a * b ));
    h = a;
    BOOST_CHECK(close_enough( h /= b,    a / b ));

    BOOST_CHECK (a == Quaternion<>(1,2,3,4) );
    BOOST_CHECK ( ! (a != Quaternion<>(1,2,3,4)) );
    BOOST_CHECK (a != Quaternion<>(0,2,3,4) );
    BOOST_CHECK (a != Quaternion<>(1,0,3,4) );
    BOOST_CHECK (a != Quaternion<>(1,2,0,4) );
    BOOST_CHECK (a != Quaternion<>(1,2,3,0) );


    Quaternion<float> af = cast<float>(a);
    for (size_t i = 0; i<4; i++)
	BOOST_CHECK((float)(a(i)) == af(i));

}
