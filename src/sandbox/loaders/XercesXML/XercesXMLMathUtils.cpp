/*
 * XercesXMLMathUtil.cpp
 *
 *  Created on: Oct 29, 2008
 *      Author: lpe
 */

#include "XercesXMLMathUtils.hpp"

#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOMNodeList.hpp>

#include <map>
#include <vector>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>


using namespace xercesc;

using namespace rw::math;
using namespace rw::common;

namespace {

bool initializeXerces() {
    try
    {
       XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    }
    catch( XMLException& e )
    {
       char* message = XMLString::transcode( e.getMessage() );
       std::cerr << "XML toolkit initialization error: " << message << std::endl;
       XMLString::release( &message );
       // throw exception here to return ERROR_XERCES_INIT
    }

    return true;
}

} //end namespace

//Small hack to make sure Xerces is initialized before XMLString::transcode is used
const bool XercesXMLMathUtils::_initialized = initializeXerces();

const XMLCh* XercesXMLMathUtils::Q_ID = XMLString::transcode("Q");
const XMLCh* XercesXMLMathUtils::Vector3D_ID = XMLString::transcode("Vector3D");
const XMLCh* XercesXMLMathUtils::Vector2D_ID = XMLString::transcode("Vector2D");

const XMLCh* XercesXMLMathUtils::Rotation3D_ID = XMLString::transcode("Rotation3D");
const XMLCh* XercesXMLMathUtils::RPY_ID = XMLString::transcode("RPY");
const XMLCh* XercesXMLMathUtils::EAA_ID = XMLString::transcode("EAA");
const XMLCh* XercesXMLMathUtils::Quaternion_ID = XMLString::transcode("Quaternion");

const XMLCh* XercesXMLMathUtils::Rotation2D_ID = XMLString::transcode("Rotation2D");
const XMLCh* XercesXMLMathUtils::Transform3D_ID = XMLString::transcode("Transform3D");
const XMLCh* XercesXMLMathUtils::VelocityScrew6D_ID = XMLString::transcode("VelocityScrew6D");

const XMLCh* XercesXMLMathUtils::Pos_ID = XMLString::transcode("Pos");
const XMLCh* XercesXMLMathUtils::Linear_ID = XMLString::transcode("Linear");
const XMLCh* XercesXMLMathUtils::Angular_ID = XMLString::transcode("Angular");


const XMLCh* XercesXMLMathUtils::Unit_Attribute_ID = XMLString::transcode("unit");

const XercesXMLMathUtils::UnitMap XercesXMLMathUtils::_Units;

XercesXMLMathUtils::UnitMap::UnitMap() {
    _map["mm"] = 1.0/1000.0;
    _map["cm"] = 1.0/100.0;
    _map["m"] = 1;
    _map["inch"] = 0.0254;

    _map["deg"] = Deg2Rad;
    _map["rad"] = 1;

    _map["m/s"] = 1;
    _map["cm/s"] = 1.0/100.0;
    _map["mm/s"] = 1.0/1000.0;

    _map["m/s^2"] = 1;
    _map["cm/s^2"] = 1.0/100.0;
    _map["mm/s^2"] = 1.0/1000.0;

    _map["deg/s"] = Deg2Rad;
    _map["rad/s"] = 1;

    _map["deg/s^2"] = Deg2Rad;
    _map["rad/s^2"] = 1;
}

double XercesXMLMathUtils::getUnit(const XMLCh* key) {
    std::string tmp = XMLString::transcode(key);
    std::map<std::string, double>::const_iterator it = _Units._map.find(tmp);
    if (it == _Units._map.end())
        RW_THROW("Invalid Unit Attribute "<<key);
    return (*it).second;

}



namespace {
    double readUnit(DOMElement* element) {
        if (element->hasAttribute(XercesXMLMathUtils::Unit_Attribute_ID)) {
            const XMLCh* attr = element->getAttribute(XercesXMLMathUtils::Unit_Attribute_ID);
            return XercesXMLMathUtils::getUnit(attr);
        } else {
            return 1;
        }
    }

    std::vector<double> parseNArray(const std::string& data) {
        std::vector<std::string> words = StringUtil::words(data);
        std::pair<bool, std::vector<double> > result = StringUtil::toDoubles(words);
        if (!result.first)
            RW_THROW("Failed to parse \""<<data<<" as a set of doubles");
        return result.second;
    }

    std::vector<double> readNVector(DOMElement* element) {
        DOMNodeList* children = element->getChildNodes();
        if (children->getLength() != 1)
            RW_THROW("Failed to parse element \""<<XMLString::transcode(element->getNodeName())<<":"<<XMLString::transcode(element->getNodeValue())<<"\". Too many child nodes");

        std::vector<double> elements = parseNArray(XMLString::transcode(children->item(0)->getNodeValue()));
        return elements;
    }


    void checkHeader(DOMElement* element, const XMLCh* id) {
        if (!XMLString::equals(element->getNodeName(), id))
            RW_THROW("Expected \""<<XMLString::transcode(id)<<"\" got "<<XMLString::transcode(element->getNodeName()));
    }

    template <class T>
    inline T readVectorStructure(DOMElement* element, bool doCheckHeader, const XMLCh* id) {

        if (doCheckHeader)
            checkHeader(element, XercesXMLMathUtils::Q_ID);

        std::cout<<"Header Ok"<<std::endl;
        double scale = 1;
        if (element->hasAttributes())
            scale = readUnit(element);
        std::cout<<"Scale = "<<scale<<std::endl;
        std::vector<double> elements = readNVector(element);

        T result;
        for (size_t i = 0; i<elements.size(); i++) {
            result(i) = scale*elements[i];
        }
        return result;
    }


} //end internal namespace

Q XercesXMLMathUtils::readQ(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Rotation3D_ID);

    std::vector<double> values = readNVector(element);
    Q q(values.size());
    for (size_t i = 0; i<values.size(); ++i)
        q(i) = values[i];

    return q;
}

Vector3D<> XercesXMLMathUtils::readVector3D(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<Vector3D<> >(element, doCheckHeader, Vector3D_ID);
}


Vector2D<> XercesXMLMathUtils::readVector2D(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<Vector2D<> >(element, doCheckHeader, Vector2D_ID);
}


RPY<> XercesXMLMathUtils::readRPY(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<RPY<> >(element, doCheckHeader, RPY_ID);
}

EAA<> XercesXMLMathUtils::readEAA(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<EAA<> >(element, doCheckHeader, EAA_ID);}

Quaternion<> XercesXMLMathUtils::readQuaternion(DOMElement* element, bool doCheckHeader) {
    Quaternion<> qua = readVectorStructure<Quaternion<> >(element, doCheckHeader, Quaternion_ID);
    qua.normalize();
    return qua;
}

Rotation3D<> XercesXMLMathUtils::readRotation3D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Rotation3D_ID);

    std::vector<double> values = readNVector(element);
    if (values.size() != 9)
        RW_THROW("Expected 9 floating points for Rotation3D. Only "<<values.size()<<" values found");

    return Rotation3D<>(values[0], values[1], values[2],
                        values[3], values[4], values[5],
                        values[6], values[7], values[8]);

}

Rotation2D<> XercesXMLMathUtils::readRotation2D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Rotation2D_ID);

    std::vector<double> values = readNVector(element);
    if (values.size() != 4)
        RW_THROW("Expected 4 floating points for Rotation3D. Only "<<values.size()<<" values found");

    return Rotation2D<>(values[0], values[1],
                        values[2], values[3]);
}

Rotation3D<> XercesXMLMathUtils::readRotation3DStructure(DOMElement* element) {
    if (XMLString::equals(element->getNodeName(), Rotation3D_ID))
        return readRotation3D(element, false);
    if (XMLString::equals(element->getNodeName(), RPY_ID))
        return readRPY(element, false).toRotation3D();
    if (XMLString::equals(element->getNodeName(), EAA_ID))
        return readEAA(element, false).toRotation3D();
    if (XMLString::equals(element->getNodeName(), Quaternion_ID))
        return readQuaternion(element, false).toRotation3D();

    RW_THROW("Unable to find match \""<<XMLString::transcode(element->getNodeName())<<"\" with (Rotation3D|RPY|EAA|Quaternion)");
}

Transform3D<> XercesXMLMathUtils::readTransform3D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Rotation3D_ID);

    Vector3D<> position(0,0,0);
    Rotation3D<> rotation(Rotation3D<>::identity());
    DOMNodeList* children = element->getChildNodes();
    const  XMLSize_t nodeCount = children->getLength();
    for (XMLSize_t i = 0; i<nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), Pos_ID)) {
                position = readVector3D(child, false);
            } else {
                rotation = readRotation3DStructure(element);
            }

        }
    }
    return Transform3D<>(position, rotation);
}

VelocityScrew6D<> XercesXMLMathUtils::readVelocityScrew6D(xercesc::DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, VelocityScrew6D_ID);

    Vector3D<> linear(0, 0, 0);
    EAA<> angular(0, 0, 0);

    DOMNodeList* children = element->getChildNodes();
    const XMLSize_t nodeCount = children->getLength();
    for (XMLSize_t i = 0; i < nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*> (children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), Linear_ID)) {
                linear = readVector3D(child, false);
            } else if (XMLString::equals(child->getNodeName(), Angular_ID)) {
                angular = readEAA(element, false);
            } else {
                RW_THROW("Unknown element \""<<XMLString::transcode(child->getNodeName())<<"\" specified in VelocityScrew6D");
            }
        }
    }

    return VelocityScrew6D<>(linear, angular);

}
