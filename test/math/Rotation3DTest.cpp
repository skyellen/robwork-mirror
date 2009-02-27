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

BOOST_AUTO_TEST_CASE(Rotation3DTest)
{
    BOOST_MESSAGE("- Testing Rotation3D");

    const Vector3D<std::string> i("i1", "i2", "i3");
    const Vector3D<std::string> j("j1", "j2", "j3");
    const Vector3D<std::string> k("k1", "k2", "k3");
    const Rotation3D<std::string> r4(i, j, k);

    BOOST_CHECK(r4(1,0) == "i2");
    BOOST_CHECK(inverse(r4)(1, 0) == "j1");

    const Rotation3D<> r1 = Rotation3D<>::identity();
    const Vector3D<> v1(1, 2, 3);
    BOOST_CHECK(norm_inf(v1 - r1 * v1) == 0);

    const EAA<> eaa(Vector3D<>(1, 0, 0), Pi / 2);
    const Rotation3D<> r3 = eaa.toRotation3D();

    BOOST_CHECK(LinearAlgebra::isSO(r3.m()));
    BOOST_CHECK(r3.m().size1() == r3.m().size2() && r3.m().size1() == 3);

    const Rotation3D<int> ri = cast<int>(r3);
    for (size_t i = 0; i < 3; i++)
        for (size_t j = 0; j < 3; j++)
            BOOST_CHECK((int)r3(i, j) == ri(i, j));
}
