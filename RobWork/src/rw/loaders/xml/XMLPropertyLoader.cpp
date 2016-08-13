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

#include "XMLPropertyLoader.hpp"

#include "XercesUtils.hpp"
#include "XMLPropertyFormat.hpp"

#include <rw/loaders/xml/XercesErrorHandler.hpp>
#include <rw/loaders/xml/XMLBasisTypes.hpp>
#include <rw/loaders/xml/XMLPathLoader.hpp>
#include <rw/loaders/xml/XMLPathFormat.hpp>



#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>
#include <xercesc/util/XMLDouble.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::trajectory;
using namespace xercesc;

XMLPropertyLoader::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		XMLBasisTypes::Initializer init1;
		XMLPropertyFormat::Initializer init2;
		XMLPathFormat::Initializer init3;
		XMLPathLoader::Initializer init4;
		done = true;
	}
}

const XMLPropertyLoader::Initializer XMLPropertyLoader::initializer;

XMLPropertyLoader::XMLPropertyLoader()
{

}

XMLPropertyLoader::~XMLPropertyLoader()
{
}

namespace {



    xercesc::DOMElement* getChildElement(xercesc::DOMElement* element, bool throwOnEmpty) {
        DOMNodeList* children = element->getChildNodes();
        for (size_t i = 0; i<children->getLength(); i++) {
            xercesc::DOMElement* element = dynamic_cast<xercesc::DOMElement*>(children->item(i));
            if (element != NULL)
                return element;
        }
		if (throwOnEmpty)
			RW_THROW("No child element to node " + XMLStr(element->getNodeName()).str());
		return NULL;
    }


	PropertyBase::Ptr getProperty(const std::string& name, const std::string& description, int type2, xercesc::DOMElement* valueNode) {		
		xercesc::DOMElement* child = getChildElement(valueNode, false);
		if (child == NULL)
			return NULL;
		
        if (XMLString::equals(child->getNodeName(), XMLPropertyFormat::idPropertyMap()))
            return ownedPtr(new Property<PropertyMap>(name, description, XMLPropertyLoader::readProperties(child, true)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idString()))
            return ownedPtr(new Property<std::string>(name, description, XMLBasisTypes::readString(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idStringList()))
            return ownedPtr(new Property<std::vector<std::string> >(name, description, XMLBasisTypes::readStringList(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idIntList()))
            return ownedPtr(new Property<std::vector<int> >(name, description, XMLBasisTypes::readIntList(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idDoubleList()))
            return ownedPtr(new Property<std::vector<double> >(name, description, XMLBasisTypes::readDoubleList(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idDouble()))
            return ownedPtr(new Property<double>(name, description, XMLBasisTypes::readDouble(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idFloat()))
            return ownedPtr(new Property<float>(name, description, XMLBasisTypes::readFloat(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idInteger()))
            return ownedPtr(new Property<int>(name, description, XMLBasisTypes::readInt(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idBoolean()))
            return ownedPtr(new Property<bool>(name, description, XMLBasisTypes::readBool(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idVector3D()))
            return ownedPtr(new Property<Vector3D<> >(name, description, XMLBasisTypes::readVector3D(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idVector2D()))
            return ownedPtr(new Property<Vector2D<> >(name, description, XMLBasisTypes::readVector2D(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idQ()))
            return ownedPtr(new Property<Q>(name, description, XMLBasisTypes::readQ(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idTransform3D()))
            return ownedPtr(new Property<Transform3D<> >(name, description, XMLBasisTypes::readTransform3D(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idRotation3D()))
            return ownedPtr(new Property<Rotation3D<> >(name, description, XMLBasisTypes::readRotation3D(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idEAA()))
            return ownedPtr(new Property<EAA<> >(name, description, XMLBasisTypes::readEAA(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idRPY()))
            return ownedPtr(new Property<RPY<> >(name, description, XMLBasisTypes::readRPY(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idQuaternion()))
            return ownedPtr(new Property<Quaternion<> >(name, description, XMLBasisTypes::readQuaternion(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idRotation2D()))
            return ownedPtr(new Property<Rotation2D<> >(name, description, XMLBasisTypes::readRotation2D(child)));
        if (XMLString::equals(child->getNodeName(), XMLBasisTypes::idVelocityScrew6D()))
            return ownedPtr(new Property<VelocityScrew6D<> >(name, description, XMLBasisTypes::readVelocityScrew6D(child)));
        if (XMLString::equals(child->getNodeName(), XMLPathFormat::idQPath())) {
           XMLPathLoader loader(child);
           return ownedPtr(new Property<QPath>(name, description, *loader.getQPath()));
        }
        if (XMLString::equals(child->getNodeName(), XMLPathFormat::idT3DPath())) {
           XMLPathLoader loader(child);
           return ownedPtr(new Property<Transform3DPath >(name, description, *loader.getTransform3DPath()));
        }

      //  switch (type) {
      // case PropertyType::PropertyMap:
      //     return ownedPtr(new Property<PropertyMap>(name, description, XMLPropertyLoader::readProperties(child, true)));
      // case PropertyType::String:
      //     return ownedPtr(new Property<std::string>(name, description, XMLBasisTypes::readString(child)));
       //case PropertyType::Float:
       //    return ownedPtr(new Property<float>(name, description, (float)XMLBasisTypes::readDouble(child)));
       //case PropertyType::Double:
       //    return ownedPtr(new Property<double>(name, description, XMLBasisTypes::readDouble(child)));
  //     case PropertyType::Int:
  //         return ownedPtr(new Property<int>(name, description, XMLBasisTypes::readInt(child)));
  //     case PropertyType::Bool:
  //         return ownedPtr(new Property<bool>(name, description, XMLBasisTypes::readBool(child)));
   //    case PropertyType::Vector3D:
   //        return ownedPtr(new Property<Vector3D<> >(name, description, XMLBasisTypes::readVector3D(child)));
   //    case PropertyType::Vector2D:
   //        return ownedPtr(new Property<Vector2D<> >(name, description, XMLBasisTypes::readVector2D(child)));
    //   case PropertyType::Q:
    //       return ownedPtr(new Property<Q>(name, description, XMLBasisTypes::readQ(child)));
       //case PropertyType::Transform3D:
       //    return ownedPtr(new Property<Transform3D<> >(name, description, XMLBasisTypes::readTransform3D(child)));
       //case PropertyType::Rotation3D:
       //    return ownedPtr(new Property<Rotation3D<> >(name, description, XMLBasisTypes::readRotation3D(child)));
       //case PropertyType::EAA:
       //    return ownedPtr(new Property<EAA<> >(name, description, XMLBasisTypes::readEAA(child)));
       //case PropertyType::RPY:
       //    return ownedPtr(new Property<RPY<> >(name, description, XMLBasisTypes::readRPY(child)));
       //case PropertyType::Quaternion:
       //    return ownedPtr(new Property<Quaternion<> >(name, description, XMLBasisTypes::readQuaternion(child)));
       //case PropertyType::Rotation2D:
       //    return ownedPtr(new Property<Rotation2D<> >(name, description, XMLBasisTypes::readRotation2D(child)));
       //case PropertyType::VelocityScrew6D:
       //    return ownedPtr(new Property<VelocityScrew6D<> >(name, description, XMLBasisTypes::readVelocityScrew6D(child)));
       //case PropertyType::QPath: {
       //    XMLPathLoader loader(child);
       //    return ownedPtr(new Property<QPath>(name, description, *loader.getQPath()));
      // }
       //case PropertyType::Transform3DPath: {
       //    XMLPathLoader loader(child);
       //    return ownedPtr(new Property<Transform3DPath >(name, description, *loader.getTransform3DPath()));
       //}


       //}//end switch (type)
	   return NULL;
   //    RW_THROW("Type of property \""+name+"\" is not supported");
   }

} //end internal namespace


PropertyBase::Ptr XMLPropertyLoader::readProperty(xercesc::DOMElement* element, bool checkHeader) {
    //std::cout<<"Read Property"<<std::endl;
    if (checkHeader)
         if (!XMLString::equals(XMLPropertyFormat::idProperty(), element->getNodeName()))
             RW_THROW("Element name does not match " + XMLStr(XMLPropertyFormat::idProperty()).str() + " as expected");

    DOMNodeList* children = element->getChildNodes();
    const  XMLSize_t nodeCount = children->getLength();

    std::string name = "";
    std::string description = "";
    int type = -1;
    xercesc::DOMElement* valueElement = NULL;
    //First we run through and finds the interpolators
    for(XMLSize_t i = 0; i < nodeCount; ++i ) {
        xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
        if (child != NULL) {
            if (XMLString::equals(XMLPropertyFormat::idPropertyName(), child->getNodeName())) {
                name = XMLBasisTypes::readString(child);
            } else if (XMLString::equals(XMLPropertyFormat::idPropertyDescription(), child->getNodeName())) {
                description = XMLBasisTypes::readString(child);
            } else if (XMLString::equals(XMLPropertyFormat::idPropertyType(), child->getNodeName())) {
                type = XMLBasisTypes::readInt(child);
            }  else if (XMLString::equals(XMLPropertyFormat::idPropertyValue(), child->getNodeName())) {
                valueElement = child;
            }
        }
    }


	/*if (type == -1) {
		RW_WARN("Unable to find type of property " << name);
    //    RW_THROW("Unable to find type of property " + name);
		return NULL;
	}*/

	if (name == "")
        RW_THROW("Unable to find name of property");
    if (valueElement == NULL)
        RW_THROW("Unable to find value of property " + name);
    return getProperty(name, description, type, valueElement);


}


PropertyMap XMLPropertyLoader::readProperties(xercesc::DOMElement* element, bool checkHeader) {

    if (checkHeader)
        if (!XMLString::equals(XMLPropertyFormat::idPropertyMap(), element->getNodeName()))
            RW_THROW("Element name does not match " + XMLStr(XMLPropertyFormat::idPropertyMap()).str() + " as expected");


    PropertyMap properties;
    DOMNodeList* children = element->getChildNodes();
    const  XMLSize_t nodeCount = children->getLength();

    //First we run through and finds the interpolators
    for(XMLSize_t i = 0; i < nodeCount; ++i ) {
        xercesc::DOMElement* element = dynamic_cast<xercesc::DOMElement*>(children->item(i));
        if (element != NULL) {
            if (XMLString::equals(XMLPropertyFormat::idProperty(), element->getNodeName())) {
				PropertyBase::Ptr property = readProperty(element, false);
				if (property != NULL)
					properties.add(property);
             }
        }
     }

    return properties;
}



PropertyMap XMLPropertyLoader::load(std::istream& instream, const std::string& schemaFileName) {
    XercesDOMParser parser;
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, instream, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement();
    PropertyMap map = readProperties(elementRoot);
    //map.set<std::string>("PropertyMapFileName", "");
    return map;
}

PropertyMap XMLPropertyLoader::load(const std::string& filename, const std::string& schemaFileName) {
	XercesDOMParser parser;
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, filename, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement();
    PropertyMap map = readProperties(elementRoot);
    //map.set<std::string>("PropertyMapFileName", filename);
    return map;
}
