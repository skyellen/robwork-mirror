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
