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

#include "PropertyMapLoader.hpp"

#include <rw/loaders/xml/XercesErrorHandler.hpp>
#include "XMLBasisTypes.hpp"
#include "PathLoader.hpp"
#include <rw/loaders/xml/XMLPathFormat.hpp>

#include <rw/common/DOMParser.hpp>

using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::trajectory;

PropertyMapLoader::PropertyMapLoader()
{

}

PropertyMapLoader::~PropertyMapLoader()
{
}

PropertyBase::Ptr PropertyMapLoader::readProperty(DOMElem::Ptr element, bool checkHeader) {
    //std::cout<<"Read Property"<<std::endl;

    if (checkHeader)
         if ( !element->isName("Property") )
             RW_THROW("Parse error: Expected \"Property\" Got \"" + element->getName() + "\"!");

	std::string name = "", description = "";
	BOOST_FOREACH( DOMElem::Ptr child, element->getChildren() ){
		if (child->isName("id") ) {
			name = child->getValue();
		} else if (child->isName("Description")) {
			description = child->getValue();
		} else if (child->isName("PropertyMap")) {
			return ownedPtr(new Property<PropertyMap>(name, description, PropertyMapLoader::readProperties(child, true)));
		} else if (child->isName("String")) {
			return ownedPtr(new Property<std::string>(name, description, XMLBasisTypes::readString(child)));
		} else if (child->isName("StringList")) {
			return ownedPtr(new Property<std::vector<std::string> >(name, description, XMLBasisTypes::readStringList(child)));
		} else if (child->isName("IntList")) {
			return ownedPtr(new Property<std::vector<int> >(name, description, XMLBasisTypes::readIntList(child)));
		} else if (child->isName("DoubleList")) {
			return ownedPtr(new Property<std::vector<double> >(name, description, XMLBasisTypes::readDoubleList(child)));
		} else if (child->isName("Double")) {
			return ownedPtr(new Property<double>(name, description, XMLBasisTypes::readDouble(child)));
		} else if (child->isName("Float")) {
			return ownedPtr(new Property<float>(name, description, XMLBasisTypes::readFloat(child)));
		} else if (child->isName("Integer")) {
			return ownedPtr(new Property<int>(name, description, XMLBasisTypes::readInt(child)));
		} else if (child->isName("Boolean")) {
			return ownedPtr(new Property<bool>(name, description, XMLBasisTypes::readBool(child)));
		} else if (child->isName("Vector3D")){
			return ownedPtr(new Property<Vector3D<> >(name, description, XMLBasisTypes::readVector3D(child)));
		} else if (child->isName("Vector2D")) {
			return ownedPtr(new Property<Vector2D<> >(name, description, XMLBasisTypes::readVector2D(child)));
		} else if (child->isName("Q")) {
			return ownedPtr(new Property<Q>(name, description, XMLBasisTypes::readQ(child)));
		} else if (child->isName("Transform3D")) {
			return ownedPtr(new Property<Transform3D<> >(name, description, XMLBasisTypes::readTransform3D(child)));
		} else if (child->isName("Rotation3D")) {
			return ownedPtr(new Property<Rotation3D<> >(name, description, XMLBasisTypes::readRotation3D(child)));
		} else if (child->isName("EAA")) {
			return ownedPtr(new Property<EAA<> >(name, description, XMLBasisTypes::readEAA(child)));
		} else if (child->isName("RPY")) {
			return ownedPtr(new Property<RPY<> >(name, description, XMLBasisTypes::readRPY(child)));
		} else if (child->isName("Quaternion")) {
			return ownedPtr(new Property<Quaternion<> >(name, description, XMLBasisTypes::readQuaternion(child)));
		} else if (child->isName("Rotation2D")) {
			return ownedPtr(new Property<Rotation2D<> >(name, description, XMLBasisTypes::readRotation2D(child)));
		} else if (child->isName("VelocityScrew6D")) {
			return ownedPtr(new Property<VelocityScrew6D<> >(name, description, XMLBasisTypes::readVelocityScrew6D(child)));
		} else if (child->isName("QPath")){
			PathLoader loader(child);
			return ownedPtr(new Property<QPath>(name, description,*loader.getQPath()));
		} else if (child->isName("T3DPath")){
			PathLoader loader(child);
			return ownedPtr(new Property<Transform3DPath >(name, description, *loader.getTransform3DPath()));
	    } else {
	    	RW_THROW("Parse Error: data value not reqognized in property with id \""<< name << "\"!");
	    }
    }

	RW_THROW("Parse Error: data value not defined in property with id \""<< name << "\"!");

	return NULL;
}


PropertyMap PropertyMapLoader::readProperties(DOMElem::Ptr element, bool checkHeader) {

    if (checkHeader)
    	if(!element->isName("PropertyMap"))
            RW_THROW("Parse error: Expected \"PropertyMap\" got \"" + element->getName() + "\"!");

    PropertyMap properties;
    BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		if ( child->isName("Property") ) {
			PropertyBase::Ptr property = readProperty(element, false);
			if (property != NULL)
				properties.add(property);
		}
     }

    return properties;
}



PropertyMap PropertyMapLoader::load(std::istream& instream, const std::string& schemaFileName) {
	DOMParser::Ptr parser = DOMParser::make();
    // todo: add schema load to interface
    parser->load(instream);
    DOMElem::Ptr elementRoot = parser->getRootElement();
    PropertyMap map = readProperties(elementRoot);
    //map.set<std::string>("PropertyMapFileName", "");
    return map;
}

PropertyMap PropertyMapLoader::load(const std::string& filename, const std::string& schemaFileName) {
	DOMParser::Ptr parser = DOMParser::make();
    // todo: add schema load to interface
    parser->load(filename);
    DOMElem::Ptr elementRoot = parser->getRootElement();
    PropertyMap map = readProperties(elementRoot);
    //map.set<std::string>("PropertyMapFileName", filename);
    return map;
}
