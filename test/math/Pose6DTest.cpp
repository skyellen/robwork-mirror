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

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Pose6D.hpp>

#include <boost/test/unit_test.hpp>

#include <iostream>

using namespace rw::math;

void Pose6DTest(){
    BOOST_MESSAGE("- Testing Pose6D");
    Pose6D<double> p(1.1,2.2,3.3,4.4,5.5,6.6);

    BOOST_CHECK(p(0) == 1.1);
    BOOST_CHECK(p(1) == 2.2);
    BOOST_CHECK(p(2) == 3.3);
    BOOST_CHECK(fabs(p(3) - 4.4)<1e-15);
    BOOST_CHECK(fabs(p(4) - 5.5)<1e-15);
    BOOST_CHECK(fabs(p(5) - 6.6)<1e-15);

    Pose6D<float> pf = cast<float>(p);
    for (size_t i = 0; i<6; i++)
	BOOST_CHECK(pf(i) == (float)p(i));
}
