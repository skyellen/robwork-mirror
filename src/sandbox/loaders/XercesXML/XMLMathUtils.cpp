#include "XMLMathUtils.hpp"
#include "XercesUtils.hpp"

#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMText.hpp>


#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

#include <sstream>
#include <map>
#include <vector>
using namespace xercesc;

using namespace rw::math;
using namespace rw::common;
using namespace rw::loaders;

namespace {

bool initializeXerces() {
    try
    {
       XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    }
    catch( XMLException& e )
    {
       RW_THROW("Unable to initialize Xerces: "<<XMLStr(e.getMessage()).str());
    }

    return true;
}

} //end namespace

//Small hack to make sure Xerces is initialized before XMLString::transcode is used
const bool XMLMathUtils::_initialized = initializeXerces();

//Definition of Identifiers used in the XML format
const XMLCh* XMLMathUtils::QId = XMLString::transcode("Q");
const XMLCh* XMLMathUtils::Vector3DId = XMLString::transcode("Vector3D");
const XMLCh* XMLMathUtils::Vector2DId = XMLString::transcode("Vector2D");

const XMLCh* XMLMathUtils::Rotation3DId = XMLString::transcode("Rotation3D");
const XMLCh* XMLMathUtils::RPYId = XMLString::transcode("RPY");
const XMLCh* XMLMathUtils::EAAId = XMLString::transcode("EAA");
const XMLCh* XMLMathUtils::QuaternionId = XMLString::transcode("Quaternion");

const XMLCh* XMLMathUtils::Rotation2DId = XMLString::transcode("Rotation2D");
const XMLCh* XMLMathUtils::Transform3DId = XMLString::transcode("Transform3D");
const XMLCh* XMLMathUtils::VelocityScrew6DId = XMLString::transcode("VelocityScrew6D");

const XMLCh* XMLMathUtils::PosId = XMLString::transcode("Pos");
const XMLCh* XMLMathUtils::MatrixId = XMLString::transcode("Matrix");

const XMLCh* XMLMathUtils::LinearId = XMLString::transcode("Linear");
const XMLCh* XMLMathUtils::AngularId = XMLString::transcode("Angular");


const XMLCh* XMLMathUtils::UnitAttributeId = XMLString::transcode("unit");

const XMLMathUtils::UnitMap XMLMathUtils::_Units;

//Setup map with units
XMLMathUtils::UnitMap::UnitMap() {
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


double XMLMathUtils::getUnit(const XMLCh* key) {
    XMLStr tmp(key);
    std::map<std::string, double>::const_iterator it = _Units._map.find(tmp.str());
    if (it == _Units._map.end())
        RW_THROW("Invalid Unit Attribute "<<tmp.str());
    return (*it).second;

}



namespace {
    double readUnit(DOMElement* element) {
        if (element->hasAttribute(XMLMathUtils::UnitAttributeId)) {
            const XMLCh* attr = element->getAttribute(XMLMathUtils::UnitAttributeId);
            return XMLMathUtils::getUnit(attr);
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
            RW_THROW("Expected \""<<XMLStr(id).str()<<"\" got "<<XMLStr(element->getNodeName()).str());
    }

    template <class T>
    inline T readVectorStructure(DOMElement* element, bool doCheckHeader, const XMLCh* id) {

        if (doCheckHeader)
            checkHeader(element, id);

        double scale = 1;
        if (element->hasAttributes())
            scale = readUnit(element);
        std::vector<double> elements = readNVector(element);

        T result;
        for (size_t i = 0; i<elements.size(); i++) {
            result(i) = scale*elements[i];
        }
        return result;
    }


} //end internal namespace

Q XMLMathUtils::readQ(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, QId);

    std::vector<double> values = readNVector(element);
    Q q(values.size());
    for (size_t i = 0; i<values.size(); ++i)
        q(i) = values[i];

    return q;
}

Vector3D<> XMLMathUtils::readVector3D(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<Vector3D<> >(element, doCheckHeader, Vector3DId);
}


Vector2D<> XMLMathUtils::readVector2D(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<Vector2D<> >(element, doCheckHeader, Vector2DId);
}


RPY<> XMLMathUtils::readRPY(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<RPY<> >(element, doCheckHeader, RPYId);
}

EAA<> XMLMathUtils::readEAA(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<EAA<> >(element, doCheckHeader, EAAId);}

Quaternion<> XMLMathUtils::readQuaternion(DOMElement* element, bool doCheckHeader) {
    Quaternion<> qua = readVectorStructure<Quaternion<> >(element, doCheckHeader, QuaternionId);
    qua.normalize();
    return qua;
}

Rotation3D<> XMLMathUtils::readRotation3D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Rotation3DId);

    std::vector<double> values = readNVector(element);
    if (values.size() != 9)
        RW_THROW("Expected 9 floating points for Rotation3D. Only "<<values.size()<<" values found");

    return Rotation3D<>(values[0], values[1], values[2],
                        values[3], values[4], values[5],
                        values[6], values[7], values[8]);

}

Rotation2D<> XMLMathUtils::readRotation2D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Rotation2DId);

    std::vector<double> values = readNVector(element);
    if (values.size() != 4)
        RW_THROW("Expected 4 floating points for Rotation3D. Only "<<values.size()<<" values found");

    return Rotation2D<>(values[0], values[1],
                        values[2], values[3]);
}

Rotation3D<> XMLMathUtils::readRotation3DStructure(DOMElement* element) {
    if (XMLString::equals(element->getNodeName(), Rotation3DId))
        return readRotation3D(element, false);
    if (XMLString::equals(element->getNodeName(), RPYId))
        return readRPY(element, false).toRotation3D();
    if (XMLString::equals(element->getNodeName(), EAAId))
        return readEAA(element, false).toRotation3D();
    if (XMLString::equals(element->getNodeName(), QuaternionId))
        return readQuaternion(element, false).toRotation3D();

    RW_THROW("Unable to find match \""<<XMLStr(element->getNodeName()).str()<<"\" with (Rotation3D|RPY|EAA|Quaternion)");
}

Transform3D<> XMLMathUtils::readTransform3D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Transform3DId);

    Vector3D<> position(0,0,0);
    Rotation3D<> rotation(Rotation3D<>::identity());
    DOMNodeList* children = element->getChildNodes();
    const  XMLSize_t nodeCount = children->getLength();
    for (XMLSize_t i = 0; i<nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), MatrixId)) {
                std::vector<double> values = readNVector(child);
                if (values.size() != 12)
                    RW_THROW("Expected <Matrix> with 12 doubles when parsing Transform3D. Found "<<values.size()<<" values");
                rotation(0,0) = values[0];
                rotation(0,1) = values[1];
                rotation(0,2) = values[2];
                rotation(1,0) = values[4];
                rotation(1,1) = values[5];
                rotation(1,2) = values[6];
                rotation(2,0) = values[8];
                rotation(2,1) = values[9];
                rotation(2,2) = values[10];

                position(0) = values[3];
                position(1) = values[7];
                position(2) = values[11];
            } else if (XMLString::equals(child->getNodeName(), PosId)) {
                position = readVector3D(child, false);
            } else {
                rotation = readRotation3DStructure(child);
            }

        }
    }
    return Transform3D<>(position, rotation);
}

VelocityScrew6D<> XMLMathUtils::readVelocityScrew6D(xercesc::DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, VelocityScrew6DId);

    Vector3D<> linear(0, 0, 0);
    EAA<> angular(0, 0, 0);

    DOMNodeList* children = element->getChildNodes();
    const XMLSize_t nodeCount = children->getLength();
    for (XMLSize_t i = 0; i < nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*> (children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), LinearId)) {
                linear = readVector3D(child, false);
            } else if (XMLString::equals(child->getNodeName(), AngularId)) {
                angular = readEAA(element, false);
            } else {
                RW_THROW("Unknown element \""<<XMLStr(child->getNodeName()).str()<<"\" specified in VelocityScrew6D");
            }
        }
    }

    return VelocityScrew6D<>(linear, angular);

}






namespace {
    template <class T>
    XMLStr createString(const T& v, size_t n) {
        std::ostringstream str;
        for (size_t i = 0; i<n; ++i) {
            str<<v(i);
            if (i != n-1)
                str<<" ";
        }
        return XMLStr(str.str());
    }

    template <class T>
    XMLStr createString(const T& v) {
        return createString<T>(v, v.size());
    }
}

DOMElement* XMLMathUtils::createQ(const Q& q, DOMDocument* doc) {
    DOMElement* element = doc->createElement(QId);
    DOMText* txt = doc->createTextNode(createString(q).uni());
    element->appendChild(txt);
    return element;
}

DOMElement* XMLMathUtils::createVector3D(const Vector3D<>& v, DOMDocument* doc) {
    DOMElement* element = doc->createElement(Vector3DId);

    std::ostringstream str;
    str<<v(0)<<" "<<v(1)<<" "<<v(2);
    DOMText* txt = doc->createTextNode(createString(v).uni());
    element->appendChild(txt);
    return element;
}

DOMElement* XMLMathUtils::createVector2D(const rw::math::Vector2D<>& v, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(Vector2DId);
    DOMText* txt = doc->createTextNode(createString(v).uni());
    element->appendChild(txt);
    return element;
}

DOMElement* XMLMathUtils::createRPY(const rw::math::RPY<>& v, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(RPYId);
    DOMText* txt = doc->createTextNode(createString(v, 3).uni());
    element->appendChild(txt);
    return element;
}

DOMElement* XMLMathUtils::createEAA(const rw::math::EAA<>& v, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(EAAId);
    DOMText* txt = doc->createTextNode(createString(v, 3).uni());
    element->appendChild(txt);
    return element;
}

DOMElement* XMLMathUtils::createQuaternion(const rw::math::Quaternion<>& q, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(QuaternionId);
    DOMText* txt = doc->createTextNode(createString(q, 4).uni());
    element->appendChild(txt);
    return element;
}


DOMElement* XMLMathUtils::createRotation3D(const rw::math::Rotation3D<>& r, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(Rotation3DId);
    std::ostringstream str;
    str<<r(0,0)<<" "<<r(0,1)<<" "<<r(0,2)<<" ";
    str<<r(1,0)<<" "<<r(1,1)<<" "<<r(1,2)<<" ";
    str<<r(2,0)<<" "<<r(2,1)<<" "<<r(2,2);
    DOMText* txt = doc->createTextNode(XMLStr(str.str()).uni());
    element->appendChild(txt);
    return element;
}

DOMElement* XMLMathUtils::createRotation2D(const rw::math::Rotation2D<>& r, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(Rotation2DId);
    std::ostringstream str;
    str<<r(0,0)<<" "<<r(0,1)<<" "<<r(1,0)<<" "<<r(1,1);
    DOMText* txt = doc->createTextNode(XMLStr(str.str()).uni());
    element->appendChild(txt);
    return element;
}

DOMElement* XMLMathUtils::createTransform3D(const rw::math::Transform3D<>& t, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(Transform3DId);

    DOMElement* posElem = doc->createElement(PosId);
    DOMText* txt = doc->createTextNode(createString(t.P()).uni());
    posElem->appendChild(txt);


    DOMElement* rotElem = createRotation3D(t.R(), doc);

    element->appendChild(posElem);
    element->appendChild(rotElem);

    return element;
}

DOMElement* XMLMathUtils::createVelocityScrew6D(const rw::math::VelocityScrew6D<>& v, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(QuaternionId);
    DOMText* txt = doc->createTextNode(createString(v, 6).uni());
    element->appendChild(txt);
    return element;
}
