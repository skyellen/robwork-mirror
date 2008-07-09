#include <iostream>


#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Constants.hpp>

#include <boost/test/unit_test.hpp>

using namespace rw::math;

void VelocityScrew6DTest() {
    BOOST_MESSAGE("- Testing VelocityScrew6D");
  {
    Transform3D<> T = Transform3D<>::identity();
    VelocityScrew6D<> screw(T);
    for (size_t i = 0; i<6; i++)
      BOOST_CHECK(screw(i)==0);
  }

  {
    Vector3D<> linear(0.1, 0.2, 0.3);
    EAA<> angular(0.4, 0.5, 0.6);
    VelocityScrew6D<> screw(linear, angular);


    BOOST_CHECK(screw.linear()(0) == linear(0));
    BOOST_CHECK(screw.linear()(1) == linear(1));
    BOOST_CHECK(screw.linear()(2) == linear(2));

    BOOST_CHECK(fabs(screw(3) - angular.axis()(0)*angular.angle())<1e-16);
    BOOST_CHECK(fabs(screw(4) - angular.axis()(1)*angular.angle())<1e-16);
    BOOST_CHECK(fabs(screw(5) - angular.axis()(2)*angular.angle())<1e-16);


  }


  Vector3D<> linear(0.6, 0.5, 0.4);
  Rotation3D<> rot3d = Rotation3D<>::identity();
  {
    RPY<> rpy(Pi/4, 0, 0);
    rot3d = rpy.toRotation3D();
    Transform3D<> T(linear, rot3d);
    VelocityScrew6D<> screw(T);
    BOOST_CHECK(screw.linear()(0) == linear(0));
    BOOST_CHECK(screw.linear()(1) == linear(1));
    BOOST_CHECK(screw.linear()(2) == linear(2));
    BOOST_CHECK(screw(3) == 0);
    BOOST_CHECK(screw(4) == 0);
    BOOST_CHECK(fabs(screw(5) - sin(Pi/4))<1e-16);
  }

  {
    RPY<> rpy(0, Pi/5, 0);
    rot3d = rpy.toRotation3D();
    Transform3D<> T(linear, rot3d);
    VelocityScrew6D<> screw(T);
    BOOST_CHECK(screw(3) == 0);
    BOOST_CHECK(fabs(screw(4) - sin(Pi/5))<1e-16);
    BOOST_CHECK(screw(5) == 0);
  }

  {
    RPY<> rpy(0, 0, Pi/6);
    rot3d = rpy.toRotation3D();
    Transform3D<> T(linear, rot3d);
    VelocityScrew6D<> screw(T);
    BOOST_CHECK(fabs(screw(3) - sin(Pi/6))<1e-16);
    BOOST_CHECK(screw(4) == 0);
    BOOST_CHECK(screw(5) == 0);
  }


  {
    Vector3D<> linear1(0.1, 0.2, 0.3);
    EAA<> angular1(0.4, 0.5, 0.6);
    VelocityScrew6D<> screw1(linear1, angular1);

    Vector3D<> linear2(0.2, 0.3, 0.4);
    EAA<> angular2(0.5, 0.6, 0.7);
    VelocityScrew6D<> screw2(linear2, angular2);

    VelocityScrew6D<> screwA(screw1);
    VelocityScrew6D<> screwB(screw2);

    screw1 += screw2;
    for (size_t i = 0; i<6; i++) {
      BOOST_CHECK(fabs(screw1(i) - (screwA(i)+screwB(i)))<1e-15);
    }

    screw2 -= screw1;
    for (size_t i = 0; i<6; i++) {
      BOOST_CHECK(fabs(screw2(i) - -screwA(i))<1e-15);
    }
  }

  {
    Vector3D<> linear1(1, 2, 3);
    EAA<> angular1(4, 5, 6);
    VelocityScrew6D<> screw(linear1, angular1);
    BOOST_CHECK(fabs(norm_1(screw)-(1+2+3+4+5+6))<1e-15);
    BOOST_CHECK(fabs(norm_2(screw)-sqrt(1.0*1+2*2+3*3+4*4+5*5+6*6))<1e-15);
    BOOST_CHECK(fabs(norm_inf(screw)-6)<1e-15);
  }


  {
    Vector3D<> linear(0.1, 0.2, 0.3);
    EAA<> angular(0.4, 0.5, 0.6);
    VelocityScrew6D<> screw(linear, angular);
    //std::cout<<"VelocityScrew<T> = "<<screw<<std::endl;
  }

  {
    Vector3D<> linear(0.1, 0.2, 0.3);
    EAA<> angular(0.4, 0.5, 0.6);
    VelocityScrew6D<> screw(linear, angular);
    VelocityScrew6D<float> vsf = cast<float>(screw);
    for (size_t i = 0; i<6; i++)
	BOOST_CHECK((float)screw(i) == vsf(i));
  }

}
