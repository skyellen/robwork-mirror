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

#include <rw/common/Property.hpp>
#include <rw/common/PropertyBase.hpp>
#include <rw/common/PropertyMap.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/foreach.hpp>

#include <iostream>

using namespace boost;
using namespace rw::common;

BOOST_AUTO_TEST_CASE( PropertyTest )
{
    //Test basic functionality
    BOOST_MESSAGE("- PropertyTest");
    Property<double>* propA = new Property<double>("A", "propA", 123.456);
    BOOST_CHECK(propA->getIdentifier() == "A");
    BOOST_CHECK(propA->getDescription() == "propA");
    BOOST_CHECK(propA->getValue() == 123.456);

    Property<std::string>* propB = new Property<std::string>("B", "propB", "HELLO");
    BOOST_CHECK(propB->getIdentifier() == "B");
    BOOST_CHECK(propB->getDescription() == "propB");
    BOOST_CHECK(propB->getValue() == "HELLO");

    PropertyBase* propPointer = propB;
    BOOST_CHECK(propPointer->getIdentifier() == "B");
    BOOST_CHECK(propPointer->getDescription() == "propB");

    PropertyMap bag;
    bag.add(
        propA->getIdentifier(),
        propA->getDescription(),
        propA->getValue());

    bag.add(
        propB->getIdentifier(),
        propB->getDescription(),
        propB->getValue());

    BOOST_CHECK(bag.size() == 2);

    BOOST_FOREACH(PropertyBasePtr prop, bag.getProperties()) {
        prop->getIdentifier();
    }

    PropertyBase* p = bag.findPropertyBase("B");
    BOOST_CHECK(p != NULL);
    BOOST_CHECK(p->getDescription() == "propB");

    Property<double>* pd = bag.findProperty<double>("A");
    BOOST_CHECK(pd != NULL);
    BOOST_CHECK(pd->getValue() == 123.456);

    // Test that NULL is returned if types do not match
    pd = bag.findProperty<double>("B");
    BOOST_CHECK(pd == NULL);
}
