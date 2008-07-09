#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.cpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Constants.hpp>

#include <iostream>

#include <boost/test/unit_test.hpp>

using namespace rw::math;

void RPYTest() {
    BOOST_MESSAGE("- Testing RPY");
    //Test default constructor
    RPY<> rpy0;
    BOOST_CHECK(rpy0(0) == 0.);
    BOOST_CHECK(rpy0(1) == 0.);
    BOOST_CHECK(rpy0(2) == 0.);


    //Test RPY(T alpha, T beta, T gamma) constructor

    RPY<> rpy1(0.1, 0.2, 0.3);
    BOOST_CHECK(rpy1(0) == 0.1);
    BOOST_CHECK(rpy1(1) == 0.2);
    BOOST_CHECK(rpy1(2) == 0.3);

    //Test toRotation3D and RPY(const Rotation3D<T>&) constructor
    Rotation3D<> rot3d = rpy1.toRotation3D();

    RPY<> rpy2(rot3d);
    BOOST_CHECK(fabs(rpy2(0)-rpy1(0))<1e-16);
    BOOST_CHECK(fabs(rpy2(1)-rpy1(1))<1e-16);
    BOOST_CHECK(fabs(rpy2(2)-rpy1(2))<1e-16);

    rpy1(0) = 0;
    rpy1(1) = Pi/2;
    rpy1(2) = 0;

    rot3d = rpy1.toRotation3D();
    BOOST_CHECK(rot3d(0,2) == 1);
    BOOST_CHECK(rot3d(1,1) == 1);
    BOOST_CHECK(rot3d(2,0) == -1);

    RPY<> rpy3(rot3d);
    BOOST_CHECK(rpy3(0) == rpy1(0));
    BOOST_CHECK(rpy3(1) == rpy1(1));
    BOOST_CHECK(rpy3(2) == rpy1(2));

    rpy1(0) = -2.5;
    rpy1(1) = 0.4;
    rpy1(2) = 4.2;
    RPY<float> rpyf = cast<float>(rpy1);
    for (size_t i = 0; i<3; i++)
        BOOST_CHECK(rpyf(i) == (float)rpy1(i));

}
