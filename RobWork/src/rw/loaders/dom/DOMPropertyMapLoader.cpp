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

#include "DOMPropertyMapLoader.hpp"

#include "DOMBasisTypes.hpp"
#include "DOMPathLoader.hpp"

#include <rw/common/DOMParser.hpp>

using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::trajectory;

DOMPropertyMapLoader::DOMPropertyMapLoader()
{

}

DOMPropertyMapLoader::~DOMPropertyMapLoader()
{
}

PropertyBase::Ptr DOMPropertyMapLoader::readProperty(DOMElem::Ptr element, bool checkHeader) {
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
			return ownedPtr(new Property<PropertyMap>(name, description, DOMPropertyMapLoader::readProperties(child, true)));
		} else if (child->isName("String")) {
			return ownedPtr(new Property<std::string>(name, description, DOMBasisTypes::readString(child)));
		} else if (child->isName("StringList")) {
			return ownedPtr(new Property<std::vector<std::string> >(name, description, DOMBasisTypes::readStringList(child)));
		} else if (child->isName("IntList")) {
			return ownedPtr(new Property<std::vector<int> >(name, description, DOMBasisTypes::readIntList(child)));
		} else if (child->isName("DoubleList")) {
			return ownedPtr(new Property<std::vector<double> >(name, description, DOMBasisTypes::readDoubleList(child)));
		} else if (child->isName("Double")) {
			return ownedPtr(new Property<double>(name, description, DOMBasisTypes::readDouble(child)));
		} else if (child->isName("Float")) {
			return ownedPtr(new Property<float>(name, description, DOMBasisTypes::readFloat(child)));
		} else if (child->isName("Integer")) {
			return ownedPtr(new Property<int>(name, description, DOMBasisTypes::readInt(child)));
		} else if (child->isName("Boolean")) {
			return ownedPtr(new Property<bool>(name, description, DOMBasisTypes::readBool(child)));
		} else if (child->isName("Vector3D")){
			return ownedPtr(new Property<Vector3D<> >(name, description, DOMBasisTypes::readVector3D(child)));
		} else if (child->isName("Vector2D")) {
			return ownedPtr(new Property<Vector2D<> >(name, description, DOMBasisTypes::readVector2D(child)));
		} else if (child->isName("Q")) {
			return ownedPtr(new Property<Q>(name, description, DOMBasisTypes::readQ(child)));
		} else if (child->isName("Transform3D")) {
			return ownedPtr(new Property<Transform3D<> >(name, description, DOMBasisTypes::readTransform3D(child)));
		} else if (child->isName("Rotation3D")) {
			return ownedPtr(new Property<Rotation3D<> >(name, description, DOMBasisTypes::readRotation3D(child)));
		} else if (child->isName("EAA")) {
			return ownedPtr(new Property<EAA<> >(name, description, DOMBasisTypes::readEAA(child)));
		} else if (child->isName("RPY")) {
			return ownedPtr(new Property<RPY<> >(name, description, DOMBasisTypes::readRPY(child)));
		} else if (child->isName("Quaternion")) {
			return ownedPtr(new Property<Quaternion<> >(name, description, DOMBasisTypes::readQuaternion(child)));
		} else if (child->isName("Rotation2D")) {
			return ownedPtr(new Property<Rotation2D<> >(name, description, DOMBasisTypes::readRotation2D(child)));
		} else if (child->isName("VelocityScrew6D")) {
			return ownedPtr(new Property<VelocityScrew6D<> >(name, description, DOMBasisTypes::readVelocityScrew6D(child)));
		} else if (child->isName("QPath")){
			DOMPathLoader loader(child);
			return ownedPtr(new Property<QPath>(name, description,*loader.getQPath()));
		} else if (child->isName("T3DPath")){
			DOMPathLoader loader(child);
			return ownedPtr(new Property<Transform3DPath >(name, description, *loader.getTransform3DPath()));
	    } else {
	    	RW_THROW("Parse Error: data value not reqognized in property with id \""<< name << "\"!");
	    }
    }

	RW_THROW("Parse Error: data value not defined in property with id \""<< name << "\"!");

	return NULL;
}


bool DOMPropertyMapLoader::hasProperties(DOMElem::Ptr element) {
    return element->isName("PropertyMap");
}

PropertyMap DOMPropertyMapLoader::readProperties(DOMElem::Ptr element, bool checkHeader) {

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



PropertyMap DOMPropertyMapLoader::load(std::istream& instream, const std::string& schemaFileName) {
	DOMParser::Ptr parser = DOMParser::make();
    // todo: add schema load to interface
    parser->load(instream);
    DOMElem::Ptr elementRoot = parser->getRootElement();
    DOMElem::Ptr pmapRoot = elementRoot->getChild("PropertyMap",false);
    PropertyMap map = readProperties(pmapRoot);
    //map.set<std::string>("PropertyMapFileName", "");
    return map;
}

PropertyMap DOMPropertyMapLoader::load(const std::string& filename, const std::string& schemaFileName) {
	DOMParser::Ptr parser = DOMParser::make();
    // todo: add schema load to interface
    parser->load(filename);
    DOMElem::Ptr elementRoot = parser->getRootElement();
    DOMElem::Ptr pmapRoot = elementRoot->getChild("PropertyMap",false);
    PropertyMap map = readProperties(pmapRoot);
    //map.set<std::string>("PropertyMapFileName", filename);
    return map;
}
