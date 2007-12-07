#include "MathTestSuite.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include <string>

using namespace boost::unit_test;

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

void testVector3D_norm(){
    Vector3D<> v1(1.0, 2.0, 2.0);

    BOOST_CHECK( norm_1(v1) == 5.0);
    BOOST_CHECK( norm_2(v1) == 3.0);
    BOOST_CHECK( norm_inf(v1) == 2.0);
}

void testVector3D_cross(){
    Vector3D<> v1(1.0, 2.0, 3.0);
    Vector3D<> v2(v1);

    BOOST_CHECK( norm_inf(cross(v1, v2)) == 0);
}

void testRotation3D_inverse(){
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

void Vector3DTest();
void Vector2DTest();
void Rotation3DTest();
void EAATest();
void Transform3DTest();
void LinearAlgebraTest();
void RPYTest();
void QuaternionTest();
void VelocityScrew6DTest();
void Pose6DTest();
void UtilTest();

MathTestSuite::MathTestSuite(){
    BOOST_MESSAGE("MathTestSuite");
    add( BOOST_TEST_CASE( &testVector3D_norm ) );
    add( BOOST_TEST_CASE( &testVector3D_cross ) );
    add( BOOST_TEST_CASE( &testRotation3D_inverse ) );

    add( BOOST_TEST_CASE( &Vector3DTest ) );
    add( BOOST_TEST_CASE( &Vector2DTest ) );
    add( BOOST_TEST_CASE( &Rotation3DTest ) );
    add( BOOST_TEST_CASE( &EAATest ) );
    add( BOOST_TEST_CASE( &Transform3DTest ) );
    add( BOOST_TEST_CASE( &LinearAlgebraTest ) );
    add( BOOST_TEST_CASE( &RPYTest ) );
    add( BOOST_TEST_CASE( &QuaternionTest ) );
    add( BOOST_TEST_CASE( &VelocityScrew6DTest ) );
    add( BOOST_TEST_CASE( &Pose6DTest ) );
    add( BOOST_TEST_CASE( &UtilTest ) );

}

