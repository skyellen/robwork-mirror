/*
 * XMLPropertySaver.cpp
 *
 *  Created on: Jan 5, 2009
 *      Author: lpe
 */

#include "XMLPropertySaver.hpp"
#include "XMLPropertyFormat.hpp"


#include <rw/math/Vector3D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

#include <rw/common/Property.hpp>

#include <rw/loaders/xml/XercesUtils.hpp>
#include <rw/loaders/xml/XMLBasisTypes.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::loaders;
using namespace xercesc;


DOMElement* XMLPropertySaver::save(PropertyBasePtr property, xercesc::DOMDocument* doc) {

    DOMElement* root = doc->createElement(XMLPropertyFormat::PropertyId);

    DOMElement* element = doc->createElement(XMLPropertyFormat::PropertyNameId);
    root->appendChild(element);
    DOMText* txt = doc->createTextNode(XMLStr(property->getIdentifier()).uni());
    element->appendChild(txt);

    element = doc->createElement(XMLPropertyFormat::PropertyDescriptionId);
    root->appendChild(element);
    txt = doc->createTextNode(XMLStr(property->getDescription()).uni());
    element->appendChild(txt);

    element = doc->createElement(XMLPropertyFormat::PropertyTypeId);
    root->appendChild(element);
    txt = doc->createTextNode(XMLStr(property->getType().getId()).uni());
    element->appendChild(txt);

    element = doc->createElement(XMLPropertyFormat::PropertyValueId);
    root->appendChild(element);

    DOMElement* elem = NULL;
    switch (property->getType().getId()) {
    case PropertyType::UNKNOWN:
        RW_WARN("Unable to save property of unknown type");
        break;
    case PropertyType::PROPERTYMAP: {
        const Property<PropertyMap>* prop = dynamic_cast<const Property<PropertyMap>*>(property.get());
        elem = save(prop->getValue(), doc);
        break;
    }
    case PropertyType::STRING: {
        const Property<std::string>* prop = dynamic_cast<const Property<std::string>*>(property.get());
        elem = XMLBasisTypes::createString(prop->getValue(), doc);
        break;
    }
    case PropertyType::FLOAT: {
        const Property<float>* prop = dynamic_cast<const Property<float>*>(property.get());
        elem = XMLBasisTypes::createDouble(prop->getValue(), doc);
        break;
    }
    case PropertyType::DOUBLE: {
        const Property<double>* prop = dynamic_cast<const Property<double>*>(property.get());
        elem = XMLBasisTypes::createDouble(prop->getValue(), doc);
        break;
    }
    case PropertyType::INT: {
        const Property<int>* prop = dynamic_cast<const Property<int>*>(property.get());
        elem = XMLBasisTypes::createInteger(prop->getValue(), doc);
        break;
    }
    case PropertyType::BOOL: {
        const Property<bool>* prop = dynamic_cast<const Property<bool>*>(property.get());
        elem = XMLBasisTypes::createBoolean(prop->getValue(), doc);
        break;
    }
    case PropertyType::VECTOR3D: {
        const Property<Vector3D<> >* prop = dynamic_cast<const Property<Vector3D<> >*>(property.get());
        elem = XMLBasisTypes::createVector3D(prop->getValue(), doc);
        break;
    }
    case PropertyType::VECTOR2D: {
        const Property<Vector2D<> >* prop = dynamic_cast<const Property<Vector2D<> >*>(property.get());
        elem = XMLBasisTypes::createVector2D(prop->getValue(), doc);
        break;
    }
    case PropertyType::Q: {
        const Property<Q>* prop = dynamic_cast<const Property<Q>*>(property.get());
        elem = XMLBasisTypes::createQ(prop->getValue(), doc);
        break;
    }
    case PropertyType::TRANSFORM3D: {
        const Property<Transform3D<> >* prop = dynamic_cast<const Property<Transform3D<> >*>(property.get());
        elem = XMLBasisTypes::createTransform3D(prop->getValue(), doc);
        break;
    }
    case PropertyType::ROTATION3D: {
        const Property<Rotation3D<> >* prop = dynamic_cast<const Property<Rotation3D<> >*>(property.get());
        elem = XMLBasisTypes::createRotation3D(prop->getValue(), doc);
        break;
    }
    case PropertyType::EAA: {
        const Property<EAA<> >* prop = dynamic_cast<const Property<EAA<> >*>(property.get());
        elem = XMLBasisTypes::createEAA(prop->getValue(), doc);
        break;
    }
    case PropertyType::RPY: {
        const Property<RPY<> >* prop = dynamic_cast<const Property<RPY<> >*>(property.get());
        elem = XMLBasisTypes::createRPY(prop->getValue(), doc);
        break;
    }
    case PropertyType::QUATERNION: {
        const Property<Quaternion<> >* prop = dynamic_cast<const Property<Quaternion<> >*>(property.get());
        elem = XMLBasisTypes::createQuaternion(prop->getValue(), doc);
        break;
    }
    case PropertyType::ROTATION2D: {
        const Property<Rotation2D<> >* prop = dynamic_cast<const Property<Rotation2D<> >*>(property.get());
        elem = XMLBasisTypes::createRotation2D(prop->getValue(), doc);
        break;
    }
    case PropertyType::VELOCITYSCREW6D: {
        const Property<VelocityScrew6D<> >* prop = dynamic_cast<const Property<VelocityScrew6D<> >*>(property.get());
        elem = XMLBasisTypes::createVelocityScrew6D(prop->getValue(), doc);
        break;
    }

    } //end switch(property.getType)


    if (elem != NULL)
        element->appendChild(elem);

    return root;
}




void XMLPropertySaver::save(const rw::common::PropertyMap& map, xercesc::DOMElement* parent, xercesc::DOMDocument* doc) {
    std::pair<PropertyMap::iterator, PropertyMap::iterator> iterators = map.getProperties();
    for (PropertyMap::iterator it = iterators.first; it != iterators.second; ++it) {
        DOMElement* element = save(*it, doc);
        parent->appendChild(element);
    }
}


DOMElement* XMLPropertySaver::save(const PropertyMap& map, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(XMLPropertyFormat::PropertyMapId);
    save(map, element, doc);
    return element;
}

void XMLPropertySaver::save(const rw::common::PropertyMap& map, const std::string& filename) {
    XMLCh* features = XMLString::transcode("Core");
    DOMImplementation* impl =  DOMImplementationRegistry::getDOMImplementation(features);
    XMLString::release(&features);

    if (impl != NULL)
    {
        try
        {
            DOMDocument* doc = impl->createDocument(0,                                  // root element namespace URI.
                                                    XMLPropertyFormat::PropertyMapId,       // root element name
                                                    0);                                 // We do not wish to specify a document type

            DOMElement* root = doc->getDocumentElement();

            //Call the save method
            save(map, root, doc);

            XercesDocumentWriter::writeDocument(doc, filename);

            doc->release();
        }
        catch (const OutOfMemoryException&)
        {
            RW_THROW("XMLPropertySaver: OutOfMemory");
        }
        catch (const DOMException& e)
        {
            RW_THROW("XMLPropertySaver: DOMException:  " << XMLString::transcode(e.getMessage()));
        }
        catch (const rw::common::Exception& exp) {
            throw exp;
        }
        catch (...)
        {
            RW_THROW("XMLPropertySaver: Unknown Exception while creating saving path");
        }
    }
    else
    {
        RW_THROW("XMLPropertySaver: Unable to find a suitable DOM Implementation");
    }

}
