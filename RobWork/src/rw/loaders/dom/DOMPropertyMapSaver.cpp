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

#include <rw/common/DOMParser.hpp>
#include <rw/common/Property.hpp>
#include <rw/common/PropertyBase.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>
#include <rw/loaders/dom/DOMPropertyMapFormat.hpp>
#include <rw/loaders/dom/DOMPathSaver.hpp>

using namespace rw::common;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::trajectory;

void DOMPropertyMapSaver::save(const PropertyBase::Ptr property, DOMElem::Ptr parent) {
    if (property->getType().getId() == PropertyType::Unknown) {
        RW_THROW("The property type is unknown and therefore not supported!");
    }

    DOMElem::Ptr root = parent->addChild(DOMPropertyMapFormat::PropertyId);
    DOMElem::Ptr element = DOMBasisTypes::createElement(DOMPropertyMapFormat::PropertyNameId, property->getIdentifier(), root);

    if (!property->getDescription().empty()) {
        element = DOMBasisTypes::createElement(DOMPropertyMapFormat::PropertyDescriptionId, property->getDescription(), root);
    }

    element = root->addChild(DOMPropertyMapFormat::PropertyValueId);
    switch (property->getType().getId()) {
    case PropertyType::Unknown:
        RW_THROW("The property type is unknown and therefore not supported! Seeing this message means there is a bug within the function throwing this!");
        break;
    case PropertyType::PropertyMap: {
        const Property<PropertyMap>* prop = toProperty<PropertyMap>(property);
        save(prop->getValue(), element);
        break;
    }
    case PropertyType::String: {
        const Property<std::string>* prop = toProperty<std::string>(property);
        DOMBasisTypes::createString(prop->getValue(), element);
        break;
    }
    case PropertyType::StringList: {
        const Property<std::vector<std::string> >* prop = toProperty<std::vector<std::string> >(property);
        DOMBasisTypes::createStringList(prop->getValue(), element);
        break;
    }
    case PropertyType::Float: {
        const Property<float>* prop = toProperty<float>(property);
        DOMBasisTypes::createFloat(prop->getValue(), element);
        break;
    }
    case PropertyType::Double: {
        const Property<double>* prop = toProperty<double>(property);
        DOMBasisTypes::createDouble(prop->getValue(), element);
        break;
    }
    case PropertyType::Int: {
        const Property<int>* prop = toProperty<int>(property);
        DOMBasisTypes::createInteger(prop->getValue(), element);
        break;
    }
    case PropertyType::Bool: {
        const Property<bool>* prop = toProperty<bool>(property);
        DOMBasisTypes::createBoolean(prop->getValue(), element);
        break;
    }
    case PropertyType::Vector3D: {
        const Property<Vector3D<> >* prop = toProperty<Vector3D<> >(property);
        DOMBasisTypes::createVector3D(prop->getValue(), element);
        break;
    }
    case PropertyType::Vector2D: {
        const Property<Vector2D<> >* prop = toProperty<Vector2D<> >(property);
        DOMBasisTypes::createVector2D(prop->getValue(), element);
        break;
    }
    case PropertyType::Q: {
        const Property<Q>* prop = toProperty<Q>(property);
        DOMBasisTypes::createQ(prop->getValue(), element);
        break;
    }
    case PropertyType::Transform3D: {
        const Property<Transform3D<> >* prop = toProperty<Transform3D<> >(property);
        DOMBasisTypes::createTransform3D(prop->getValue(), element);
        break;
    }
    case PropertyType::Rotation3D: {
        const Property<Rotation3D<> >* prop = toProperty<Rotation3D<> >(property);
        DOMBasisTypes::createRotation3D(prop->getValue(), element);
        break;
    }
    case PropertyType::EAA: {
        const Property<EAA<> >* prop = toProperty<EAA<> >(property);
        DOMBasisTypes::createEAA(prop->getValue(), element);
        break;
    }
    case PropertyType::RPY: {
        const Property<RPY<> >* prop = toProperty<RPY<> >(property);
        DOMBasisTypes::createRPY(prop->getValue(), element);
        break;
    }
    case PropertyType::Quaternion: {
        const Property<Quaternion<> >* prop = toProperty<Quaternion<> >(property);
        Quaternion<> normalizedQuaternion(prop->getValue());
        normalizedQuaternion.normalize();
        DOMBasisTypes::createQuaternion(normalizedQuaternion, element);
        break;
    }
    case PropertyType::Rotation2D: {
        const Property<Rotation2D<> >* prop = toProperty<Rotation2D<> >(property);
        DOMBasisTypes::createRotation2D(prop->getValue(), element);
        break;
    }
    case PropertyType::VelocityScrew6D: {
        const Property<VelocityScrew6D<> >* prop = toProperty<VelocityScrew6D<> >(property);
        DOMBasisTypes::createVelocityScrew6D(prop->getValue(), element);
        break;
    }
    case PropertyType::IntList: {
        const Property<std::vector<int> >* prop = toProperty<std::vector<int> >(property);
        DOMBasisTypes::createIntList(prop->getValue(), element);
        break;
    }
    case PropertyType::DoubleList: {
        const Property<std::vector<double> >* prop = toProperty<std::vector<double> >(property);
        DOMBasisTypes::createDoubleList(prop->getValue(), element);
        break;
    }
    case PropertyType::QPath: {
        const Property<QPath>* prop = toProperty<QPath>(property);
        DOMPathSaver::createQPath(prop->getValue(), element);
        break;
    }
    case PropertyType::Transform3DPath: {
        const Property<Transform3DPath>* prop = toProperty<Transform3DPath>(property);
        DOMPathSaver::createTransform3DPath(prop->getValue(), element);
        break;
    }
    default:
        RW_THROW("The property type has no save implementation within DOMPropertyMapSaver!");
    } //end switch(property.getType)

}

void DOMPropertyMapSaver::save(const rw::common::PropertyMap& map, DOMElem::Ptr parent) {
    DOMElem::Ptr root = parent->addChild(DOMPropertyMapFormat::PropertyMapId);

    std::pair<PropertyMap::iterator, PropertyMap::iterator> iterators = map.getProperties();
    for (PropertyMap::iterator it = iterators.first; it != iterators.second; ++it) {
        save(*it, root);
    }
}

void DOMPropertyMapSaver::save(const rw::common::PropertyMap& map, const std::string& filename) {
    /* DOMParser::make() as of this writing returns the default XML parser */
    DOMParser::Ptr parser = DOMParser::make();

    createDOMDocument(map, parser);
    parser->save(filename);
}

void DOMPropertyMapSaver::write(const rw::common::PropertyMap& map, std::ostream& outstream) {
    /* DOMParser::make() as of this writing returns the default XML parser */
    DOMParser::Ptr parser = DOMParser::make();

    createDOMDocument(map, parser);
    parser->save(outstream);
}

DOMElem::Ptr DOMPropertyMapSaver::createDOMDocument(const rw::common::PropertyMap& map, DOMParser::Ptr parser) {
    DOMElem::Ptr doc = parser->getRootElement();

    save(map, doc);

    return doc;
}
