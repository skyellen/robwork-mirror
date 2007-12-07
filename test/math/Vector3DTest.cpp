#include <rw/math/Vector3D.hpp>

#include <boost/test/unit_test.hpp>

using namespace rw::math;

namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return norm_inf(v.m());
    }

    double norm_2(const Vector3D<>& v)
    {
        return norm_2(v.m());
    }
}

void Vector3DTest(){
    Vector3D<> v1(1.0, 2.0, 3.0);
    Vector3D<> v2(v1);
    Vector3D<> v3 = v1 + v2;
    Vector3D<> v4 = Vector3D<>(2.0, 4.0, 6.0);
    BOOST_CHECK( norm_inf( v3 - v4 ) == 0);

    Vector3D<> v5(3.0, 4.0, 5.0);
    Vector3D<> v5_norm = normalize(v5);
    BOOST_CHECK(fabs(norm_2(v5_norm)-1)<1e-15);
    double l = norm_2(v5);
    BOOST_CHECK(fabs(v5_norm(0) - v5(0)/l)<1e-15);
    BOOST_CHECK(fabs(v5_norm(1) - v5(1)/l)<1e-15);
    BOOST_CHECK(fabs(v5_norm(2) - v5(2)/l)<1e-15);

    v5(0) = l;
    BOOST_CHECK(v5(0) == l);


    Vector3D<std::string> vs1("x1", "y1", "z1");
    //  Vector3D<std::string> vs2("x2", "y2", "z2");

    BOOST_CHECK(vs1[0] == "x1");
    BOOST_CHECK(vs1[1] == "y1");
    BOOST_CHECK(vs1[2] == "z1");


    BOOST_CHECK( norm_inf(cross(v1, v2)) == 0);

    Vector3D<double> vd(1.1, 5.51, -10.3);
    Vector3D<int> vi = cast<int>(vd);
    BOOST_CHECK(vi(0) == 1);
    BOOST_CHECK(vi(1) == 5);
    BOOST_CHECK(vi(2) == -10);


}
