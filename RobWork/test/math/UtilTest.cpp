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


#include <rw/math/EAA.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>

#include <boost/test/unit_test.hpp>

using namespace rw::math;

BOOST_AUTO_TEST_CASE(UtilTest)
{
    BOOST_MESSAGE("- Testing Utils");
    Quaternion<> q(0.1, 0.2, 0.3, 0.4);
    q.normalize();
    const EAA<> eaa(1,2,3);

    const EAA<> eaa_q = Math::quaternionToEAA(q);
    const Quaternion<> q_eaa = Math::eaaToQuaternion(eaa_q);

    BOOST_CHECK_CLOSE(q(0),q_eaa(0), 1e-12);
    BOOST_CHECK_CLOSE(q(1),q_eaa(1), 1e-12);
    BOOST_CHECK_CLOSE(q(2),q_eaa(2), 1e-12);
    BOOST_CHECK_CLOSE(q(3),q_eaa(3), 1e-12);
}

BOOST_AUTO_TEST_CASE(testCeilLog2)
{
    BOOST_CHECK_EQUAL(Math::ceilLog2(1), 0);
    BOOST_CHECK_EQUAL(Math::ceilLog2(2), 1);
    BOOST_CHECK_EQUAL(Math::ceilLog2(3), 2);
    BOOST_CHECK_EQUAL(Math::ceilLog2(4), 2);
    BOOST_CHECK_EQUAL(Math::ceilLog2(5), 3);
    BOOST_CHECK_EQUAL(Math::ceilLog2(8), 3);
    BOOST_CHECK_EQUAL(Math::ceilLog2(9), 4);
}

BOOST_AUTO_TEST_CASE(testVector3D_norm){
    Vector3D<> v1(1.0, 2.0, 2.0);

    BOOST_CHECK_EQUAL( MetricUtil::norm1(v1), 5.0);
    BOOST_CHECK_EQUAL( MetricUtil::norm2(v1), 3.0);
    BOOST_CHECK_EQUAL( MetricUtil::normInf(v1), 2.0);
}

BOOST_AUTO_TEST_CASE(testVector3D_cross){
    Vector3D<> v1(1.0, 2.0, 3.0);
    Vector3D<> v2(v1);

    BOOST_CHECK_EQUAL( MetricUtil::normInf(cross(v1, v2)), 0);
}

BOOST_AUTO_TEST_CASE(testRotation3D_inverse){
    Vector3D<std::string> i("i1", "i2", "i3");
    Vector3D<std::string> j("j1", "j2", "j3");
    Vector3D<std::string> k("k1", "k2", "k3");
    Rotation3D<std::string> rot(i, j, k);
    BOOST_CHECK(rot(1,0) == "i2");
    BOOST_CHECK(inverse(rot)(1,0) == "j1");

    Rotation3D<std::string> rotInv(inverse(rot));
    for(int r=0;r<3;r++){
        for(int c=0;c<3;c++){
            BOOST_CHECK(rot(r,c) == rotInv(c,r));
        }
    }
}
