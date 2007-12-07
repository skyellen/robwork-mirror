#include <rw/common/Property.hpp>
#include <rw/common/PropertyBase.hpp>
#include <rw/common/PropertyMap.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/shared_ptr.hpp>

#include <iostream>

using namespace boost;
using namespace rw::common;

void PropertyTest(){
    //Test basic functionality
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
    bag.addProperty(shared_ptr<Property<double> >(propA));
    bag.addProperty(shared_ptr<Property<std::string> >(propB));
    BOOST_CHECK(bag.size() == 2);
    std::vector<shared_ptr<PropertyBase> > props = bag.properties();
    for (std::vector<shared_ptr<PropertyBase> >::iterator it = props.begin(); it != props.end(); ++it)
	std::cout<<(*it)->getIdentifier()<<std::endl;
    BOOST_CHECK(props.size() == 2);
    BOOST_CHECK(props[0]->getIdentifier() == "A");
    BOOST_CHECK(props[1]->getIdentifier() == "B");
    

    PropertyBase* p = bag.find("B");
    BOOST_CHECK(p != NULL);
    BOOST_CHECK(p->getDescription() == "propB");

    Property<double>* pd = bag.getProperty<double>("A");
    BOOST_CHECK(pd != NULL);
    BOOST_CHECK(pd->getValue() == 123.456);

    //Test that NULL is returned if types does not match
    pd = bag.getProperty<double>("B");
    BOOST_CHECK(pd == NULL);




}
