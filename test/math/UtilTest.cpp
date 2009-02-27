/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include <rw/math/EAA.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Math.hpp>

#include <boost/test/unit_test.hpp>

using namespace rw::math;

namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return norm_inf(v.m());
    }

    double norm_1(const Vector3D<>& v)
    {
        return norm_1(v.m());
    }

    double norm_2(const Vector3D<>& v)
    {
        return norm_2(v.m());
    }
}


BOOST_AUTO_TEST_CASE(UtilTest)
{
    BOOST_MESSAGE("- Testing Utils");
    Quaternion<> q(0.1, 0.2, 0.3, 0.4);
    q.normalize();
    const EAA<> eaa(1,2,3);

    const EAA<> eaa_q = Math::quaternionToEAA(q);
    const Quaternion<> q_eaa = Math::eaaToQuaternion(eaa_q);

    BOOST_CHECK(fabs(q(0) - q_eaa(0)) < 1e-12);
    BOOST_CHECK(fabs(q(1) - q_eaa(1)) < 1e-12);
    BOOST_CHECK(fabs(q(2) - q_eaa(2)) < 1e-12);
    BOOST_CHECK(fabs(q(3) - q_eaa(3)) < 1e-12);
}

BOOST_AUTO_TEST_CASE(testCeilLog2)
{
    BOOST_CHECK(Math::ceilLog2(1) == 0);
    BOOST_CHECK(Math::ceilLog2(2) == 1);
    BOOST_CHECK(Math::ceilLog2(3) == 2);
    BOOST_CHECK(Math::ceilLog2(4) == 2);
    BOOST_CHECK(Math::ceilLog2(5) == 3);
    BOOST_CHECK(Math::ceilLog2(8) == 3);
    BOOST_CHECK(Math::ceilLog2(9) == 4);
}

BOOST_AUTO_TEST_CASE(testVector3D_norm){
    Vector3D<> v1(1.0, 2.0, 2.0);

    BOOST_CHECK( norm_1(v1) == 5.0);
    BOOST_CHECK( norm_2(v1) == 3.0);
    BOOST_CHECK( norm_inf(v1) == 2.0);
}

BOOST_AUTO_TEST_CASE(testVector3D_cross){
    Vector3D<> v1(1.0, 2.0, 3.0);
    Vector3D<> v2(v1);

    BOOST_CHECK( norm_inf(cross(v1, v2)) == 0);
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
