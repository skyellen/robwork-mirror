#include "XMLBasisTypes.hpp"
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
using namespace rw::kinematics;
using namespace rw::models;
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
const bool XMLBasisTypes::_initialized = initializeXerces();

//Definition of Identifiers used in the XML format
const XMLCh* XMLBasisTypes::QId = XMLString::transcode("Q");
const XMLCh* XMLBasisTypes::Vector3DId = XMLString::transcode("Vector3D");
const XMLCh* XMLBasisTypes::Vector2DId = XMLString::transcode("Vector2D");

const XMLCh* XMLBasisTypes::Rotation3DId = XMLString::transcode("Rotation3D");
const XMLCh* XMLBasisTypes::RPYId = XMLString::transcode("RPY");
const XMLCh* XMLBasisTypes::EAAId = XMLString::transcode("EAA");
const XMLCh* XMLBasisTypes::QuaternionId = XMLString::transcode("Quaternion");

const XMLCh* XMLBasisTypes::Rotation2DId = XMLString::transcode("Rotation2D");
const XMLCh* XMLBasisTypes::Transform3DId = XMLString::transcode("Transform3D");
const XMLCh* XMLBasisTypes::VelocityScrew6DId = XMLString::transcode("VelocityScrew6D");

const XMLCh* XMLBasisTypes::PosId = XMLString::transcode("Pos");
const XMLCh* XMLBasisTypes::MatrixId = XMLString::transcode("Matrix");

const XMLCh* XMLBasisTypes::LinearId = XMLString::transcode("Linear");
const XMLCh* XMLBasisTypes::AngularId = XMLString::transcode("Angular");

const XMLCh* XMLBasisTypes::StateId = XMLString::transcode("State");
const XMLCh* XMLBasisTypes::QStateId = XMLString::transcode("QState");
const XMLCh* XMLBasisTypes::TreeStateId = XMLString::transcode("TreeState");

const XMLCh* XMLBasisTypes::BooleanId = XMLString::transcode("Boolean");
const XMLCh* XMLBasisTypes::DoubleId = XMLString::transcode("Double");
const XMLCh* XMLBasisTypes::IntegerId = XMLString::transcode("Integer");
const XMLCh* XMLBasisTypes::StringId = XMLString::transcode("String");
const XMLCh* XMLBasisTypes::StringPairId = XMLString::transcode("StringPair");


const XMLCh* XMLBasisTypes::UnitAttributeId = XMLString::transcode("unit");

const XMLBasisTypes::UnitMap XMLBasisTypes::_Units;

//Setup map with units
XMLBasisTypes::UnitMap::UnitMap() {
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


double XMLBasisTypes::getUnit(const XMLCh* key) {
    XMLStr tmp(key);
    std::map<std::string, double>::const_iterator it = _Units._map.find(tmp.str());
    if (it == _Units._map.end())
        RW_THROW("Invalid Unit Attribute "<<tmp.str());
    return (*it).second;

}



namespace {
    double readUnit(DOMElement* element) {
        if (element->hasAttribute(XMLBasisTypes::UnitAttributeId)) {
            const XMLCh* attr = element->getAttribute(XMLBasisTypes::UnitAttributeId);
            return XMLBasisTypes::getUnit(attr);
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

        std::vector<double> elements = parseNArray(XMLStr(children->item(0)->getNodeValue()).str());
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

Q XMLBasisTypes::readQ(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, QId);

    std::vector<double> values = readNVector(element);
    Q q(values.size());
    for (size_t i = 0; i<values.size(); ++i)
        q(i) = values[i];

    return q;
}

Vector3D<> XMLBasisTypes::readVector3D(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<Vector3D<> >(element, doCheckHeader, Vector3DId);
}


Vector2D<> XMLBasisTypes::readVector2D(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<Vector2D<> >(element, doCheckHeader, Vector2DId);
}


RPY<> XMLBasisTypes::readRPY(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<RPY<> >(element, doCheckHeader, RPYId);
}

EAA<> XMLBasisTypes::readEAA(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<EAA<> >(element, doCheckHeader, EAAId);}

Quaternion<> XMLBasisTypes::readQuaternion(DOMElement* element, bool doCheckHeader) {
    Quaternion<> qua = readVectorStructure<Quaternion<> >(element, doCheckHeader, QuaternionId);
    qua.normalize();
    return qua;
}

Rotation3D<> XMLBasisTypes::readRotation3D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Rotation3DId);

    std::vector<double> values = readNVector(element);
    if (values.size() != 9)
        RW_THROW("Expected 9 floating points for Rotation3D. Only "<<values.size()<<" values found");

    return Rotation3D<>(values[0], values[1], values[2],
                        values[3], values[4], values[5],
                        values[6], values[7], values[8]);

}

Rotation2D<> XMLBasisTypes::readRotation2D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Rotation2DId);

    std::vector<double> values = readNVector(element);
    if (values.size() != 4)
        RW_THROW("Expected 4 floating points for Rotation3D. Only "<<values.size()<<" values found");

    return Rotation2D<>(values[0], values[1],
                        values[2], values[3]);
}

Rotation3D<> XMLBasisTypes::readRotation3DStructure(DOMElement* element) {
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

Transform3D<> XMLBasisTypes::readTransform3D(DOMElement* element, bool doCheckHeader) {
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

VelocityScrew6D<> XMLBasisTypes::readVelocityScrew6D(xercesc::DOMElement* element, bool doCheckHeader) {
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


/**
 * @brief Returns rw::kinematics::State<> element read from \b element
 *
 * Read in \b element and returns a rw::kinematics::State corresponding to the content.
 * If \b doCheckHeader = true it checks that the elements tag name matches State.
 * If the name does not an exception is thrown.
 *
 * @param element [in] Element to read
 * @param doCheckHeader [in] True if the header name should be checked
 * @return The element read
 */
rw::kinematics::State XMLBasisTypes::readState(xercesc::DOMElement* element, WorkCellPtr workcell, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, StateId);

    State result = workcell->getDefaultState();
    DOMNodeList* children = element->getChildNodes();
    const XMLSize_t nodeCount = children->getLength();
    for (XMLSize_t i = 0; i < nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*> (children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), QStateId)) {
                Q q = readQ(child, false);
                if (result.size() != q.size())
                    RW_THROW("Length of State loaded does not match workcell");
                for (size_t i = 0; i<q.size(); i++)
                    result(i) = q(i);

            } else if (XMLString::equals(child->getNodeName(), TreeStateId)) {
                std::vector<StringPair> dafs = readStringPairs(element);
                for (std::vector<StringPair>::iterator it = dafs.begin(); it != dafs.end(); ++it) {
                    Frame* daf = workcell->findFrame((*it).first);
                    Frame* parent = workcell->findFrame((*it).second);
                    if (daf == NULL)
                        RW_THROW("Unable to locate frame named \""<<(*it).first);
                    if (parent == NULL)
                        RW_THROW("Unable to locate frame named \""<<(*it).second);

                    daf->attachTo(parent, result);
                }
            } else {
                RW_THROW("Unknown element \""<<XMLStr(child->getNodeName()).str()<<"\" specified in State");
            }
        }
    }
    return result;
}

std::string XMLBasisTypes::readString(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, StringId);

    return readElementText(element);

}

XMLBasisTypes::StringPair XMLBasisTypes::readStringPair(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, StringPairId);

    DOMNodeList* children = element->getChildNodes();
    const XMLSize_t nodeCount = children->getLength();
    std::vector<std::string> result;
    for (XMLSize_t i = 0; i < nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*> (children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), StringPairId)) {
                std::string str = readString(child, true);
                result.push_back(str);
            }
        }
    }

    if (result.size() != 2)
        RW_THROW("Expected 2 string in StringPair but found"<<result.size());

    return std::make_pair(result[0], result[1]);

}

std::vector<XMLBasisTypes::StringPair> XMLBasisTypes::readStringPairs(DOMElement* element) {
    DOMNodeList* children = element->getChildNodes();
    const XMLSize_t nodeCount = children->getLength();
    std::vector<StringPair> result;
    for (XMLSize_t i = 0; i < nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*> (children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), StringPairId)) {
                std::string str = readString(child);
                std::vector<std::string> strings = StringUtil::words(str);
                if (strings.size() != 2)
                    RW_THROW("Expected two string elements found "<<strings.size()<<" in \""<<str<<"\"");
                result.push_back(std::make_pair(strings[0], strings[1]));
            }
        }
    }
    return result;
}



std::string XMLBasisTypes::readElementText(xercesc::DOMElement* element, bool exceptionOnEmpty) {
    DOMNodeList* children = element->getChildNodes();

    for (size_t i = 0; i<children->getLength(); i++) {
        DOMNode* child = children->item(i);
        std::cout<<"readElementText Child = "<<XMLStr(child->getNodeName()).str()<<std::endl;
        if (dynamic_cast<DOMText*>(children->item(0)) != NULL)
            return XMLStr(child->getNodeValue()).str();
    }
    if (exceptionOnEmpty)
        RW_THROW("Unable to find value in Node " + XMLStr(element->getNodeName()).str());
    return "";
}

double XMLBasisTypes::readDouble(xercesc::DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, DoubleId);

    return std::atof(readElementText(element).c_str());
}


int XMLBasisTypes::readInt(xercesc::DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, IntegerId);

    return std::atoi(readElementText(element).c_str());
}

bool XMLBasisTypes::readBool(xercesc::DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, BooleanId);

    std::string str = readElementText(element);
    return str == "true";
}


namespace {
    template <class T>
    XMLStr createStringFromArray(const T& v, size_t n) {
        std::ostringstream str;
        for (size_t i = 0; i<n; ++i) {
            str<<v(i);
            if (i != n-1)
                str<<" ";
        }
        return XMLStr(str.str());
    }

    template <class T>
    XMLStr createStringFromArray(const T& v) {
        return createStringFromArray<T>(v, v.size());
    }

}
    DOMElement* XMLBasisTypes::createElement(const XMLCh* id, const XMLCh* value, DOMDocument* doc) {
        DOMElement* element = doc->createElement(id);
        DOMText* txt = doc->createTextNode(value);
        element->appendChild(txt);
        return element;
     }



DOMElement* XMLBasisTypes::createQ(const Q& q, DOMDocument* doc) {
    return createElement(QId, createStringFromArray(q).uni(), doc);
    /*DOMElement* element = doc->createElement(QId);
    DOMText* txt = doc->createTextNode(createStringFromArray(q).uni());
    element->appendChild(txt);
    return element;*/
}

DOMElement* XMLBasisTypes::createVector3D(const Vector3D<>& v, DOMDocument* doc) {
    return createElement(Vector3DId, createStringFromArray(v).uni(), doc);
    /*DOMElement* element = doc->createElement(Vector3DId);

    std::ostringstream str;
    str<<v(0)<<" "<<v(1)<<" "<<v(2);
    DOMText* txt = doc->createTextNode(createVectorString(v).uni());
    element->appendChild(txt);
    return element;*/
}

DOMElement* XMLBasisTypes::createVector2D(const rw::math::Vector2D<>& v, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(Vector2DId);
    DOMText* txt = doc->createTextNode(createStringFromArray(v).uni());
    element->appendChild(txt);
    return element;
}

DOMElement* XMLBasisTypes::createRPY(const rw::math::RPY<>& v, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(RPYId);
    DOMText* txt = doc->createTextNode(createStringFromArray(v, 3).uni());
    element->appendChild(txt);
    return element;
}

DOMElement* XMLBasisTypes::createEAA(const rw::math::EAA<>& v, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(EAAId);
    DOMText* txt = doc->createTextNode(createStringFromArray(v, 3).uni());
    element->appendChild(txt);
    return element;
}

DOMElement* XMLBasisTypes::createQuaternion(const rw::math::Quaternion<>& q, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(QuaternionId);
    DOMText* txt = doc->createTextNode(createStringFromArray(q, 4).uni());
    element->appendChild(txt);
    return element;
}


DOMElement* XMLBasisTypes::createRotation3D(const rw::math::Rotation3D<>& r, xercesc::DOMDocument* doc) {
    //DOMElement* element = doc->createElement(Rotation3DId);
    std::ostringstream str;
    str<<r(0,0)<<" "<<r(0,1)<<" "<<r(0,2)<<" ";
    str<<r(1,0)<<" "<<r(1,1)<<" "<<r(1,2)<<" ";
    str<<r(2,0)<<" "<<r(2,1)<<" "<<r(2,2);
    return createElement(Rotation3DId, XMLStr(str.str()).uni(), doc);
    /*DOMText* txt = doc->createTextNode(XMLStr(str.str()).uni());
    element->appendChild(txt);
    return element;*/
}

DOMElement* XMLBasisTypes::createRotation2D(const rw::math::Rotation2D<>& r, xercesc::DOMDocument* doc) {

    //DOMElement* element = doc->createElement(Rotation2DId);
    std::ostringstream str;
    str<<r(0,0)<<" "<<r(0,1)<<" "<<r(1,0)<<" "<<r(1,1);
    return createElement(Rotation2DId, XMLStr(str.str()).uni(), doc);
    /*DOMText* txt = doc->createTextNode(XMLStr(str.str()).uni());
    element->appendChild(txt);
    return element;*/
}

DOMElement* XMLBasisTypes::createTransform3D(const rw::math::Transform3D<>& t, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(Transform3DId);

    DOMElement* posElem = createElement(PosId, createStringFromArray(t.P()).uni(), doc);
    /*DOMElement* posElem = doc->createElement(PosId);
    DOMText* txt = doc->createTextNode(createStringFromArray(t.P()).uni());
    posElem->appendChild(txt);*/


    DOMElement* rotElem = createRotation3D(t.R(), doc);

    element->appendChild(posElem);
    element->appendChild(rotElem);

    return element;
}

DOMElement* XMLBasisTypes::createVelocityScrew6D(const rw::math::VelocityScrew6D<>& v, xercesc::DOMDocument* doc) {
    return createElement(VelocityScrew6DId, createStringFromArray(v, 6).uni(), doc);
    /*DOMElement* element = doc->createElement(VelocityScrew6DId);
    DOMText* txt = doc->createTextNode(createStringFromArray(v, 6).uni());
    element->appendChild(txt);
    return element;*/
}


DOMElement* XMLBasisTypes::createQState(const rw::kinematics::State& state, xercesc::DOMDocument* doc) {
    return createElement(QStateId, createStringFromArray<State>(state, state.size()).uni(), doc);

    /*DOMElement* element = doc->createElement(QStateId);
    DOMText* txt = doc->createTextNode(createStringFromArray<State>(state, state.size()).uni());
    element->appendChild(txt);
    return element;*/
}


DOMElement* XMLBasisTypes::createBoolean(bool value, xercesc::DOMDocument* doc) {
    return createElement(BooleanId, XMLStr(value).uni(), doc);
}

DOMElement* XMLBasisTypes::createDouble(double value, xercesc::DOMDocument* doc) {
    return createElement(DoubleId, XMLStr(value).uni(), doc);
}

DOMElement* XMLBasisTypes::createInteger(int value, xercesc::DOMDocument* doc) {
    return createElement(IntegerId, XMLStr(value).uni(), doc);
}



DOMElement* XMLBasisTypes::createString(const std::string& str, DOMDocument* doc) {
    return createElement(StringId, XMLStr(str).uni(), doc);
    /*DOMElement* element = doc->createElement(StringId);
    DOMText* txt = doc->createTextNode(XMLStr(str).uni());
    element->appendChild(txt);
    return element;*/
}

DOMElement* XMLBasisTypes::createStringPair(const std::string& first, const std::string& second, DOMDocument* doc) {
    DOMElement* element = doc->createElement(StringPairId);
    element->appendChild(createString(first, doc));
    element->appendChild(createString(second, doc));
    return element;
}

DOMElement* XMLBasisTypes::createTreeState(const rw::kinematics::State& state, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(TreeStateId);

    const std::vector<Frame*>& dafs = state.getStateStructure()->getDAFs();


    // Find out what frames are DAFs.
    typedef std::vector<Frame*>::const_iterator I;
    for (I p = dafs.begin(); p != dafs.end(); ++p) {
        Frame* frame = *p;
        DOMElement* stringpair = createStringPair(frame->getName(), frame->getDafParent(state)->getName(), doc);
        element->appendChild(stringpair);
    }
    return element;
}

DOMElement* XMLBasisTypes::createState(const rw::kinematics::State& state, xercesc::DOMDocument* doc) {
    DOMElement* element = doc->createElement(StateId);
    element->appendChild(createQState(state, doc));
    element->appendChild(createTreeState(state, doc));
    return element;
}


