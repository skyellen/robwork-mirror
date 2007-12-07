#include <rw/math/Quaternion.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

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

void QuaternionTest(){

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


 /*
  close_enough( QuatF(1,0,0,0) * QuatF(1,0,0,0), QuatF(1,0,0,0) ); // quat * quat
  close_enough( QuatF(0,1,0,0) * QuatF(0,1,0,0), QuatF(-1,0,0,0) );
  close_enough( QuatF(0,0,1,0) * QuatF(0,0,1,0), QuatF(-1,0,0,0) );
  close_enough( QuatF(0,0,0,1) * QuatF(0,0,0,1), QuatF(-1,0,0,0) );

  close_enough( QuatD(1,0,0,0) * QuatD(0,1,0,0), QuatD(0,1,0,0) );
  close_enough( QuatD(1,0,0,0) * QuatD(0,0,1,0), QuatD(0,0,1,0) );
  close_enough( QuatD(1,0,0,0) * QuatD(0,0,0,1), QuatD(0,0,0,1) );
  close_enough( QuatD(0,1,0,0) * QuatD(0,0,1,0), QuatD(0,0,0,1) );
  close_enough( QuatD(0,1,0,0) * QuatD(0,0,0,1), QuatD(0,0,-1,0) );
  close_enough( QuatD(0,0,1,0) * QuatD(0,0,0,1), QuatD(0,1,0,0) );
  close_enough( QuatD(0,1,0,0) * QuatD(1,0,0,0), QuatD(0,1,0,0) );
  close_enough( QuatD(0,0,1,0) * QuatD(1,0,0,0), QuatD(0,0,1,0) );
  close_enough( QuatD(0,0,0,1) * QuatD(1,0,0,0), QuatD(0,0,0,1) );
  close_enough( QuatD(0,0,1,0) * QuatD(0,1,0,0), QuatD(0,0,0,-1) );
  close_enough( QuatD(0,0,0,1) * QuatD(0,1,0,0), QuatD(0,0,1,0) );
  close_enough( QuatD(0,0,0,1) * QuatD(0,0,1,0), QuatD(0,-1,0,0) );
*/
}
