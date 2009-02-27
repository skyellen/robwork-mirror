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

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Constants.hpp>

#include <iostream>

#include <boost/test/unit_test.hpp>

using namespace rw::math;

namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return norm_inf(v.m());
    }
}

BOOST_AUTO_TEST_CASE( EAATest ){
    BOOST_MESSAGE("- Testing EAA");
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
    BOOST_CHECK(fabs(e180_2.angle() - Pi)<1e-16);
    BOOST_CHECK(norm_inf(e180_2.axis() - Vector3D<>(0.0, 1.0, 0.0)) == 0);
    EAA<> xe180_2(e180_2.toRotation3D());
    BOOST_CHECK(fabs(xe180_2.angle() - e180_2.angle())<1e-16);
    BOOST_CHECK(norm_inf(xe180_2.axis() - e180_2.axis())==0);

    EAA<> e180_3(0.0, 0.0, Pi);
    BOOST_CHECK(fabs(e180_3.angle() - Pi)<1e-16);
    BOOST_CHECK(norm_inf(e180_3.axis() - Vector3D<>(0.0, 0.0, 1.0)) == 0);
    EAA<> xe180_3(e180_3.toRotation3D());
    BOOST_CHECK(fabs(xe180_3.angle() - e180_3.angle())<1e-16);
    BOOST_CHECK(norm_inf(xe180_3.axis() - e180_3.axis())==0);

    // 90 degree's around x axis
    Vector3D<> v1(1.0, 0.0, 0.0);
    EAA<> e1(v1, Pi / 2.0);
    BOOST_CHECK(fabs(e1.angle() - Pi / 2.0)<1e-16);
    BOOST_CHECK(norm_inf( e1.axis() - v1 ) == 0);
    Rotation3D<> rot = e1.toRotation3D();
    EAA<> e2(rot);
    BOOST_CHECK(norm_inf( e1.axis() - e2.axis() ) == 0);
    BOOST_CHECK( fabs(e1.angle() - e2.angle())<1e-16 );
}
