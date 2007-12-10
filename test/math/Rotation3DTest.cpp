#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Constants.hpp>

#include <iostream>
#include <string>

#include <boost/test/unit_test.hpp>

using namespace rw::math;

namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return norm_inf(v.m());
    }
}

void Rotation3DTest(){
    Rotation3D<> r1 = Rotation3D<>::Identity();
    Rotation3D<> r2 = Rotation3D<>::Identity();
    Rotation3D<> r3 = Rotation3D<>::Identity();

    Vector3D<std::string> i("i1", "i2", "i3");
    Vector3D<std::string> j("j1", "j2", "j3");
    Vector3D<std::string> k("k1", "k2", "k3");
    Rotation3D<std::string> r4(i, j, k);

    BOOST_CHECK( r4(1,0) == "i2" );
    BOOST_CHECK( inverse(r4)(1, 0) == "j1" );

    Vector3D<> v1(1.0, 2.0, 3.0);
    BOOST_CHECK( norm_inf( v1 - r1 * v1 ) == 0);

    EAA<> eaa( Vector3D<>(1.0, 0.0, 0.0), Pi/2.0);
    r3 = eaa.toRotation3D();

    BOOST_CHECK(LinearAlgebra::IsSO(r3.m()));
    BOOST_CHECK(r3.m().size1() == r3.m().size2() && r3.m().size1() == 3);


    Rotation3D<int> ri = cast<int>(r3);
    for (size_t i = 0; i<3; i++)
    for (size_t j = 0; j<3; j++)
        BOOST_CHECK((int)r3(i,j) == ri(i,j));

}
