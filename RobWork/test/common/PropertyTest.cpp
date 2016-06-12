/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "../TestSuiteConfig.hpp"

#include <rw/common/Property.hpp>
#include <rw/common/PropertyBase.hpp>
#include <rw/common/PropertyMap.hpp>

#include <boost/foreach.hpp>

#include <iostream>

using namespace boost;
using namespace rw::common;

BOOST_AUTO_TEST_CASE( PropertyTest )
{
    //Test basic functionality
	BOOST_TEST_MESSAGE("- PropertyTest");
    Property<double>::Ptr propA = ownedPtr(new Property<double>("A", "propA", 123.456));
    BOOST_CHECK(propA->getIdentifier() == "A");
    BOOST_CHECK(propA->getDescription() == "propA");
    BOOST_CHECK(propA->getValue() == 123.456);

    Property<std::string>::Ptr propB = ownedPtr( new Property<std::string>("B", "propB", "HELLO") );
    BOOST_CHECK(propB->getIdentifier() == "B");
    BOOST_CHECK(propB->getDescription() == "propB");
    BOOST_CHECK(propB->getValue() == "HELLO");

    PropertyBase *propPointer = propB.get();
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

    BOOST_FOREACH(PropertyBase::Ptr prop, bag.getProperties()) {
        std::string str = prop->getIdentifier();
        BOOST_CHECK(str!="");
    }

    PropertyBase::Ptr p = bag.findPropertyBase("B");
    BOOST_CHECK(p != NULL);
    BOOST_CHECK(p->getDescription() == "propB");

    Property<double>::Ptr pd = bag.findProperty<double>("A");
    BOOST_CHECK(pd != NULL);
    BOOST_CHECK(pd->getValue() == 123.456);

    // Test that NULL is returned if types do not match
    pd = bag.findProperty<double>("B");
    BOOST_CHECK(pd == NULL);
}
