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

#include "XMLPropertyFormat.hpp"

#include <rw/loaders/xml/XercesErrorHandler.hpp>
#include <rw/loaders/xml/XMLBasisTypes.hpp>
#include <rw/loaders/xml/XMLPathLoader.hpp>



#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>
#include <xercesc/util/XMLDouble.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::trajectory;
using namespace xercesc;

XMLPropertyLoader::XMLPropertyLoader()
{

}

XMLPropertyLoader::~XMLPropertyLoader()
{
}

namespace {



    DOMElement* getChildElement(DOMElement* element) {
        DOMNodeList* children = element->getChildNodes();
        for (size_t i = 0; i<children->getLength(); i++) {
            DOMElement* element = dynamic_cast<DOMElement*>(children->item(i));
            if (element != NULL)
                return element;
        }
        RW_THROW("No child element to node " + XMLStr(element->getNodeName()).str());
    }


   PropertyBasePtr getProperty(const std::string& name, const std::string& description, int type, DOMElement* valueNode) {

       DOMElement* child = getChildElement(valueNode);
       switch (type) {
       case PropertyType::PropertyMap:
           return ownedPtr(new Property<PropertyMap>(name, description, XMLPropertyLoader::readProperties(child, true)));
       case PropertyType::String:
           return ownedPtr(new Property<std::string>(name, description, XMLBasisTypes::readString(child)));
       case PropertyType::Float:
           return ownedPtr(new Property<float>(name, description, XMLBasisTypes::readDouble(child)));
       case PropertyType::Double:
           return ownedPtr(new Property<double>(name, description, XMLBasisTypes::readDouble(child)));
       case PropertyType::Int:
           return ownedPtr(new Property<int>(name, description, XMLBasisTypes::readInt(child)));
       case PropertyType::Bool:
           return ownedPtr(new Property<bool>(name, description, XMLBasisTypes::readBool(child)));
       case PropertyType::Vector3D:
           return ownedPtr(new Property<Vector3D<> >(name, description, XMLBasisTypes::readVector3D(child)));
       case PropertyType::Vector2D:
           return ownedPtr(new Property<Vector2D<> >(name, description, XMLBasisTypes::readVector2D(child)));
       case PropertyType::Q:
           return ownedPtr(new Property<Q>(name, description, XMLBasisTypes::readQ(child)));
       case PropertyType::Transform3D:
           return ownedPtr(new Property<Transform3D<> >(name, description, XMLBasisTypes::readTransform3D(child)));
       case PropertyType::Rotation3D:
           return ownedPtr(new Property<Rotation3D<> >(name, description, XMLBasisTypes::readRotation3D(child)));
       case PropertyType::EAA:
           return ownedPtr(new Property<EAA<> >(name, description, XMLBasisTypes::readEAA(child)));
       case PropertyType::RPY:
           return ownedPtr(new Property<RPY<> >(name, description, XMLBasisTypes::readRPY(child)));
       case PropertyType::Quaternion:
           return ownedPtr(new Property<Quaternion<> >(name, description, XMLBasisTypes::readQuaternion(child)));
       case PropertyType::Rotation2D:
           return ownedPtr(new Property<Rotation2D<> >(name, description, XMLBasisTypes::readRotation2D(child)));
       case PropertyType::VelocityScrew6D:
           return ownedPtr(new Property<VelocityScrew6D<> >(name, description, XMLBasisTypes::readVelocityScrew6D(child)));
       case PropertyType::QPath: {
           XMLPathLoader loader(child);
           return ownedPtr(new Property<QPath>(name, description, *loader.getQPath()));
       }
       case PropertyType::Transform3DPath: {
           XMLPathLoader loader(child);
           return ownedPtr(new Property<Transform3DPath >(name, description, *loader.getTransform3DPath()));
       }

       }//end switch (type)
       RW_THROW("Type of property \""+name+"\" is not supported");
   }

} //end internal namespace


PropertyBasePtr XMLPropertyLoader::readProperty(DOMElement* element, bool checkHeader) {
    //std::cout<<"Read Property"<<std::endl;
    if (checkHeader)
         if (!XMLString::equals(XMLPropertyFormat::PropertyId, element->getNodeName()))
             RW_THROW("Element name does not match " + XMLStr(XMLPropertyFormat::PropertyId).str() + " as expected");

    DOMNodeList* children = element->getChildNodes();
    const  XMLSize_t nodeCount = children->getLength();

    std::string name = "";
    std::string description = "";
    int type = -1;
    DOMElement* valueElement = NULL;
    //First we run through and finds the interpolators
    for(XMLSize_t i = 0; i < nodeCount; ++i ) {
        DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
        if (child != NULL) {
            if (XMLString::equals(XMLPropertyFormat::PropertyNameId, child->getNodeName())) {
                name = XMLBasisTypes::readString(child);
            } else if (XMLString::equals(XMLPropertyFormat::PropertyDescriptionId, child->getNodeName())) {
                description = XMLBasisTypes::readString(child);
            } else if (XMLString::equals(XMLPropertyFormat::PropertyTypeId, child->getNodeName())) {
                type = XMLBasisTypes::readInt(child);
            }  else if (XMLString::equals(XMLPropertyFormat::PropertyValueId, child->getNodeName())) {
                valueElement = child;
            }
        }
    }

    if (name == "")
        RW_THROW("Unable to find name of property");
    if (type == -1)
        RW_THROW("Unable to find type of property " + name);
    if (valueElement == NULL)
        RW_THROW("Unable to find value of property " + name);
    return getProperty(name, description, type, valueElement);


}


PropertyMap XMLPropertyLoader::readProperties(DOMElement* element, bool checkHeader) {

    if (checkHeader)
        if (!XMLString::equals(XMLPropertyFormat::PropertyMapId, element->getNodeName()))
            RW_THROW("Element name does not match " + XMLStr(XMLPropertyFormat::PropertyMapId).str() + " as expected");


    PropertyMap properties;
    DOMNodeList* children = element->getChildNodes();
    const  XMLSize_t nodeCount = children->getLength();

    //First we run through and finds the interpolators
    for(XMLSize_t i = 0; i < nodeCount; ++i ) {
        DOMElement* element = dynamic_cast<DOMElement*>(children->item(i));
        if (element != NULL) {
            if (XMLString::equals(XMLPropertyFormat::PropertyId, element->getNodeName())) {
                PropertyBasePtr property = readProperty(element, false);
                properties.add(property);
             }
        }
     }

    return properties;
}



PropertyMap XMLPropertyLoader::load(const std::string& filename, const std::string& schemaFileName) {
    try
    {
        XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    }
    catch( XMLException& e )
    {
        RW_THROW("Xerces initialization Error"<<XMLStr(e.getMessage()).str());
    }

    XercesDOMParser parser;

    XercesErrorHandler errorHandler;

    parser.setDoNamespaces( true );
    parser.setDoSchema( true );
    if (schemaFileName.size() != 0)
        parser.setExternalNoNamespaceSchemaLocation(schemaFileName.c_str());


    parser.setErrorHandler(&errorHandler);
    parser.setValidationScheme(XercesDOMParser::Val_Auto);

    parser.parse(filename.c_str() );
    if (parser.getErrorCount() != 0) {
        RW_THROW(""<<parser.getErrorCount()<<" Errors: "<<errorHandler.getMessages());
    }


    // no need to free this pointer - owned by the parent parser object
    DOMDocument* xmlDoc = parser.getDocument();

    // Get the top-level element: NAme is "root". No attributes for "root"
    DOMElement* elementRoot = xmlDoc->getDocumentElement();

    return readProperties(elementRoot);
}
