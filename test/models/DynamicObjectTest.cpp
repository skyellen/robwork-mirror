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
#include <rw/models/DynamicObject.hpp>

#include <boost/test/unit_test.hpp>

using namespace rw;
using namespace rw::models;
using namespace rw::math;

void DynamicObjectTest() {

    Transform3D<> transform = Transform3D<>::Identity();
    //Test Constructor
    InertiaMatrix inertia;
    for (size_t i = 0; i<inertia.size1(); i++)
    for (size_t j = 0; j<inertia.size2(); j++)
        inertia(i,j) = 10*i+j;

    DynamicObject object(NULL, "DynObject", transform, 1.23f, inertia);
    BOOST_CHECK(object.getMass() == 1.23f);
    for (size_t i = 0; i<inertia.size1(); i++)
    for (size_t j = 0; j<inertia.size2(); j++)
        BOOST_CHECK(object.getInertia()(i,j) == inertia(i,j));


    //Test set methods
    InertiaMatrix inertia2;
    for (size_t i = 0; i<inertia2.size1(); i++)
    for (size_t j = 0; j<inertia2.size2(); j++)
        inertia2(i,j) = 1+10*i+j;
    object.setInertia(inertia2);
    object.setMass(2.46);


    BOOST_CHECK(object.getMass() == 2.46f);
    for (size_t i = 0; i<inertia2.size1(); i++)
    for (size_t j = 0; j<inertia2.size2(); j++)
        BOOST_CHECK(object.getInertia()(i,j) == inertia2(i,j));


}
