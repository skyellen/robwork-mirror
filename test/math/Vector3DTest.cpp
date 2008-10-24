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

void Vector3DTest()
{
    BOOST_MESSAGE("- Testing Vector3D");
    const Vector3D<> v1(1.0, 2.0, 3.0);
    const Vector3D<> v2(v1);
    const Vector3D<> v3 = v1 + v2;
    const Vector3D<> v4 = Vector3D<>(2.0, 4.0, 6.0);
    BOOST_CHECK( norm_inf( v3 - v4 ) == 0);

    const Vector3D<> v5(3.0, 4.0, 5.0);
    const Vector3D<> v5_norm = normalize(v5);
    BOOST_CHECK(fabs(norm_2(v5_norm)-1)<1e-15);
    const double len = norm_2(v5);
    BOOST_CHECK(fabs(v5_norm(0) - v5(0)/len)<1e-15);
    BOOST_CHECK(fabs(v5_norm(1) - v5(1)/len)<1e-15);
    BOOST_CHECK(fabs(v5_norm(2) - v5(2)/len)<1e-15);

    Vector3D<> v6;
    v6(0) = len;
    BOOST_CHECK(v6(0) == len);

    const Vector3D<std::string> vs1("x1", "y1", "z1");

    BOOST_CHECK(vs1[0] == "x1");
    BOOST_CHECK(vs1[1] == "y1");
    BOOST_CHECK(vs1[2] == "z1");

    BOOST_CHECK( norm_inf(cross(v1, v2)) == 0);

    const Vector3D<double> vd(1.1, 5.51, -10.3);
    const Vector3D<int> vi = cast<int>(vd);
    BOOST_CHECK(vi(0) == 1);
    BOOST_CHECK(vi(1) == 5);
    BOOST_CHECK(vi(2) == -10);
}
