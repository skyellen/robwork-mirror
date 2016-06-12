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

#include <rw/math/Quaternion.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>
#include <iostream>
#include <rw/common/Timer.hpp>
#include <math.h>
#include <boost/math/special_functions/sign.hpp>


using namespace rw::math;
typedef Quaternion<double> QuatD;	// just abbreviations
typedef Quaternion<float>  QuatF;


bool close_enough(Quaternion<> q1,Quaternion<> q2){
    return fabs((q1).getQx()-(q2).getQx())<1e-16 &&
    fabs((q1).getQy()-(q2).getQy())<1e-16 &&
    fabs((q1).getQz()-(q2).getQz())<1e-16 &&
    fabs((q1).getQw()-(q2).getQw())<1e-16;
}

Quaternion<> toQuatN( const Rotation3D<>& rot){

    double d = sqrt( std::max( 0.0, 1 + rot(0,0) + rot(1,1) + rot(2,2) ) ) / 2;
    double a = sqrt( std::max( 0.0, 1 + rot(0,0) - rot(1,1) - rot(2,2) ) ) / 2;
    double b = sqrt( std::max( 0.0, 1 - rot(0,0) + rot(1,1) - rot(2,2) ) ) / 2;
    double c = sqrt( std::max( 0.0, 1 - rot(0,0) - rot(1,1) + rot(2,2) ) ) / 2;
    a = boost::math::copysign( a, rot(2,1) - rot(1,2) );
    b = boost::math::copysign( b, rot(0,2) - rot(2,0) );
    c = boost::math::copysign( c, rot(1,0) - rot(0,1) );

    return Quaternion<>(a,b,c,d);
}

Quaternion<> toQuatN2( const Rotation3D<>& rot){
    double a,b,c,d;
    double tr = 1.0;

    if( rot(0,0) > rot(1,1) ){
        tr -= rot(1,1);
        if( rot(0,0) > rot(2,2) ){
            // 00 was biggest
            tr += rot(0,0) - rot(2,2);
            tr = sqrt(tr);
            d = (rot(1,2)-rot(2,1))/(2*tr);
            a = tr/2;
            b = (rot(0,2)+rot(2,0))/(2*tr);
            c = (rot(1,0)+rot(0,1))/(2*tr);
        } else {
            // 22 was biggest
            tr += rot(2,2) - rot(0,0);
            tr = sqrt(tr);
            d = (rot(1,0)-rot(0,1))/(2*tr);
            a = tr/2;
            b = (rot(2,0)+rot(0,2))/(2*tr);
            c = (rot(1,2)+rot(2,1))/(2*tr);
        }
    } else {
        tr -= rot(0,0);
        if( rot(1,1) > rot(2,2) ){
            // 11 is biggest
            tr += rot(1,1) - rot(2,2);
            tr = sqrt(tr);
            d = (rot(2,0)-rot(0,2))/(2*tr);
            a = tr/2;
            b = (rot(1,0)+rot(0,1))/(2*tr);
            c = (rot(2,1)+rot(1,2))/(2*tr);

        } else {
            // 22 is biggest
            tr += rot(2,2) -rot(1,1);
            tr = sqrt(tr);
            d = (rot(1,0)-rot(0,1))/(2*tr);
            a = tr/2;
            b = (rot(2,0)+rot(0,2))/(2*tr);
            c = (rot(1,2)+rot(2,1))/(2*tr);
        }
    }



    return Quaternion<>(a,b,c,d);
}

/*
Quaternion<> toQuatN2( const Rotation3D<>& rot){
    double a,b,c,d;
    if( rot(0,0) > rot(1,1) ){

    }
    const double tr = static_cast<double>(rot(0, 0) + rot(1, 1) + rot(2, 2) + 1);

    if (tr > 1e-5) {
        const double s = static_cast<double>(0.5) / static_cast<double>(sqrt(tr));
        d = static_cast<double>(0.25) / s;
        a = static_cast<double>(rot(2, 1) - rot(1, 2)) * s;
        b = static_cast<double>(rot(0, 2) - rot(2, 0)) * s;
        c = static_cast<double>(rot(1, 0) - rot(0, 1)) * s;
    } else {
    }
}*/
Quaternion<> toQuat( const Rotation3D<>& rot){
    double a,b,c,d;
    const double tr = static_cast<double>(rot(0, 0) + rot(1, 1) + rot(2, 2) + 1);

    if (tr > 1e-7) {
        const double s = static_cast<double>(0.5) / static_cast<double>(sqrt(tr));
        d = static_cast<double>(0.25) / s;
        a = static_cast<double>(rot(2, 1) - rot(1, 2)) * s;
        b = static_cast<double>(rot(0, 2) - rot(2, 0)) * s;
        c = static_cast<double>(rot(1, 0) - rot(0, 1)) * s;
    } else {
        if (rot(0, 0) > rot(1, 1) && rot(0, 0) > rot(2, 2)) {
            const double sa = static_cast<double>(sqrt(rot(0, 0) - rot(1, 1) - rot(2, 2) + 1.0));
            a = static_cast<double>(0.5)  *  sa;

            // s == 1 / (2.0  *  sa) == 0.25 / (0.5  *  sa)
            const double s = static_cast<double>(0.25) / a;
            b = static_cast<double>(rot(0, 1) + rot(1, 0)) * s;
            c = static_cast<double>(rot(0, 2) + rot(2, 0)) * s;
            d = static_cast<double>(rot(1, 2) - rot(2, 1)) * s;
        } else if (rot(1, 1) > rot(2, 2)) {
            const double sb = static_cast<double>(sqrt(rot(1, 1) - rot(2, 2) - rot(0, 0)  + 1));
            b = static_cast<double>(0.5) * sb;

            const double s = static_cast<double>(0.25) / b;
            a = static_cast<double>(rot(0, 1) + rot(1, 0)) * s;
            c = static_cast<double>(rot(1, 2) + rot(2, 1)) * s;
            d = static_cast<double>(rot(0, 2) - rot(2, 0)) * s;
        } else {
            const double sc = static_cast<double>(sqrt(rot(2, 2) - rot(0, 0) - rot(1, 1)  + 1));
            c = static_cast<double>(0.5) * sc;

            const double s = static_cast<double>(0.25) / c;
            a = static_cast<double>(rot(0, 2) + rot(2, 0)) * s;
            b = static_cast<double>(rot(1, 2) + rot(2, 1)) * s;
            d = static_cast<double>(rot(0, 1) - rot(1, 0)) * s;
        }

    }
    return Quaternion<>(a,b,c,d);

}


BOOST_AUTO_TEST_CASE(QuaternionConversionTest){
    BOOST_TEST_MESSAGE("- Testing Quaternion - Rotation conversion test");
    // we generate a large amount of random rotations and and convert to and from rotation3d
    const size_t count = 100000;
    std::vector<Rotation3D<> > rotations(count);
    for(size_t i=0;i<count; i++){
        Rotation3D<> rot = Math::ranRotation3D<double>();
        for(size_t j=0; j<30; j++)
            rot = rot * Math::ranRotation3D<double>();
        if(i==0)
            rot = RPY<>(0,0,0).toRotation3D();
        rotations[i] = rot;
    }
    const double epsilon = 0.00000001;
    rw::common::Timer time;
    //for(size_t m=0;m<100; m++)
    for(size_t i=0;i<count; i++){
        Rotation3D<> rot = rotations[i];
        Quaternion<> q( rot );
        Rotation3D<> res = q.toRotation3D();

        BOOST_CHECK_MESSAGE(MetricUtil::dist2(res*Vector3D<>::z(), rot*Vector3D<>::z() )<epsilon, i << " : " << MetricUtil::dist2(res*Vector3D<>::z(), rot*Vector3D<>::z() ) << " q:" << q);
        BOOST_REQUIRE(MetricUtil::dist2(res*Vector3D<>::y(), rot*Vector3D<>::y() )<epsilon);
        BOOST_REQUIRE(MetricUtil::dist2(res*Vector3D<>::x(), rot*Vector3D<>::x() )<epsilon);
    }

    time.pause();
    //std::cout << "time1: " << time.getTime() << std::endl;

}

BOOST_AUTO_TEST_CASE(QuaternionTest){
    BOOST_TEST_MESSAGE("- Testing Quaternion");

    //std::cout << "after \n" << Quaternion<>(r3d_after) << std::endl;
    //std::cout << "RPY bfore \n" << RPY<>(r3d) << std::endl;
    //std::cout << "RPY after \n" << RPY<>(r3d_after) << std::endl<<std::endl;

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
    //BOOST_CHECK(close_enough( h /= b,    a / b ));

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
