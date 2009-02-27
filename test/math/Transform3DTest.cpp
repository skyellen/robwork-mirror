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

#include <rw/math/Transform3D.hpp>

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Constants.hpp>

#include <boost/test/unit_test.hpp>

using namespace rw::math;

namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return norm_inf(v.m());
    }
}

BOOST_AUTO_TEST_CASE(Transform3DTest){
    BOOST_MESSAGE("- Testing Transform3D");
    const Vector3D<> d(1, 2, 3);
    const Transform3D<> t(d);

    const Vector3D<> v1(0, 0, 0);
    BOOST_CHECK(norm_inf(t * v1 - d) == 0);

    const EAA<> eaa(Pi / 2, 0, 0);
    const Rotation3D<> r1 = eaa.toRotation3D();
    const Transform3D<> t2(d, r1);
    const Transform3D<> t3 = t2 * inverse(t2);

    const EAA<> xeaa(t3.R());
    BOOST_CHECK(norm_inf(xeaa.axis()) == 0);
    BOOST_CHECK(xeaa.angle() == 0);
    BOOST_CHECK(norm_inf(t3.P()) < 1e-15);

    const Transform3D<float> tf = cast<float>(t);
    for (size_t i = 0; i < 3; i++)
        for (size_t j = 0; j < 4; j++)
            BOOST_CHECK(tf(i, j) == (float)t(i, j));
}
