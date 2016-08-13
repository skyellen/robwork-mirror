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

#include "XMLBasisTypes.hpp"
#include "XercesUtils.hpp"

#include <xercesc/util/XMLString.hpp>

#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMText.hpp>
#include <xercesc/util/XMLDouble.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/models/WorkCell.hpp>
#include <sstream>
#include <map>
#include <vector>
using namespace xercesc;

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::loaders;

XMLBasisTypes::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		//Small trick to make sure Xerces is initialized before we start using XMLString::transcode
		static XercesInitializer initializer;
		idQ();
		idVector3D();
		idVector2D();
		idRotation3D();
		idRPY();
		idEAA();
		idQuaternion();
		idRotation2D();
		idTransform3D();
		idVelocityScrew6D();
		idPos();
		idMatrix();
		idLinear();
		idAngular();
		idState();
		idQState();
		idTreeState();
		idBoolean();
		idDouble();
		idFloat();
		idInteger();
		idString();
		idStringList();
		idIntList();
		idDoubleList();
		idStringPair();
		idUnitAttribute();
		done = true;
	}
}

const XMLBasisTypes::Initializer XMLBasisTypes::initializer;

//Definition of Identifiers used in the XML format
const XMLCh* XMLBasisTypes::idQ() {
	static const XMLStr id("Q");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idVector3D() {
	static const XMLStr id("Vector3D");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idVector2D() {
	static const XMLStr id("Vector2D");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idRotation3D() {
	static const XMLStr id("Rotation3D");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idRPY() {
	static const XMLStr id("RPY");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idEAA() {
	static const XMLStr id("EAA");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idQuaternion() {
	static const XMLStr id("Quaternion");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idRotation2D() {
	static const XMLStr id("Rotation2D");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idTransform3D() {
	static const XMLStr id("Transform3D");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idVelocityScrew6D() {
	static const XMLStr id("VelocityScrew6D");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idPos() {
	static const XMLStr id("Pos");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idMatrix() {
	static const XMLStr id("Matrix");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idLinear() {
	static const XMLStr id("Linear");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idAngular() {
	static const XMLStr id("Angular");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idState() {
	static const XMLStr id("State");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idQState() {
	static const XMLStr id("QState");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idTreeState() {
	static const XMLStr id("TreeState");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idBoolean() {
	static const XMLStr id("Boolean");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idDouble() {
	static const XMLStr id("Double");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idFloat() {
	static const XMLStr id("Float");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idInteger() {
	static const XMLStr id("Integer");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idString() {
	static const XMLStr id("String");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idStringList() {
	static const XMLStr id("StringList");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idIntList() {
	static const XMLStr id("IntList");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idDoubleList() {
	static const XMLStr id("DoubleList");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idStringPair() {
	static const XMLStr id("StringPair");
	return id.uni();
}

const XMLCh* XMLBasisTypes::idUnitAttribute() {
	static const XMLStr id("unit");
	return id.uni();
}

namespace {
struct UnitMap {
public:
    std::map<std::string, double> _map;

    UnitMap(){
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
    };
    UnitMap(const std::map<std::string, double>& map):
    _map(map)
    {
    }

    ~UnitMap(){
    }
};
}

//const XMLBasisTypes::UnitMap XMLBasisTypes::_Units;
const UnitMap _Units;



double XMLBasisTypes::getUnit(const XMLCh* key) {
    XMLStr tmp(key);
    std::map<std::string, double>::const_iterator it = _Units._map.find(tmp.str());
    if (it == _Units._map.end())
        RW_THROW("Invalid Unit Attribute "<<tmp.str());
    return (*it).second;

}



namespace {
    double readUnit(DOMElement* element) {
        if (element->hasAttribute(XMLBasisTypes::idUnitAttribute())) {
            const XMLCh* attr = element->getAttribute(XMLBasisTypes::idUnitAttribute());
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
        XMLSize_t len = children->getLength();
        if( len == 0){
            return std::vector<double>();
        } else if (len != 1)
            RW_THROW("Failed to parse element \""
                    << XMLStr(element->getNodeName()).str()
                    <<":"
                    <<XMLStr(element->getNodeValue()).str()
                    <<"\". Too many child nodes. Nr children: "
                    << children->getLength());

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
        checkHeader(element, idQ());

    std::vector<double> values = readNVector(element);
    Q q(values.size());
    for (size_t i = 0; i<values.size(); ++i)
        q(i) = values[i];

    return q;
}

Vector3D<> XMLBasisTypes::readVector3D(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<Vector3D<> >(element, doCheckHeader, idVector3D());
}


Vector2D<> XMLBasisTypes::readVector2D(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<Vector2D<> >(element, doCheckHeader, idVector2D());
}


RPY<> XMLBasisTypes::readRPY(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<RPY<> >(element, doCheckHeader, idRPY());
}

EAA<> XMLBasisTypes::readEAA(DOMElement* element, bool doCheckHeader) {
    return readVectorStructure<EAA<> >(element, doCheckHeader, idEAA());}

Quaternion<> XMLBasisTypes::readQuaternion(DOMElement* element, bool doCheckHeader) {
    Quaternion<> qua = readVectorStructure<Quaternion<> >(element, doCheckHeader, idQuaternion());
    qua.normalize();
    return qua;
}

Rotation3D<> XMLBasisTypes::readRotation3D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, idRotation3D());

    std::vector<double> values = readNVector(element);
    if (values.size() != 9)
        RW_THROW("Expected 9 floating points for Rotation3D. Only "<<values.size()<<" values found");

    Rotation3D<> rot(values[0], values[1], values[2],
                        values[3], values[4], values[5],
                        values[6], values[7], values[8]);

    Eigen::MatrixXd u, v;
	Eigen::VectorXd w;
    LinearAlgebra::svd(rot.e(), u, w ,v);
	Eigen::MatrixXd res = u * v.transpose();

    rot = Rotation3D<>(res);

    using namespace boost::numeric::ublas;
    /*
    */

	while(fabs(rot.e().determinant()-1.0)>0.00001  ){
        std::cout.precision(16);
        std::cout << rot << std::endl;
        RW_WARN("Parse of Rotation3D failed. A rotation 3d must be an "
			"orthogonal matrix with determinant of 1! det=" << rot.e().determinant());
        LinearAlgebra::svd(rot.e(), u, w ,v);
        res = u*v.transpose();
        rot = Rotation3D<>(res);

    }
/*
    int cnt =0;
    while(fabs(LinearAlgebra::det(rot.m())-1.0)>0.00001  ){
        if(cnt>3){
            std::cout << rot << std::endl;
            RW_THROW("Parse of Rotation3D failed. A rotation 3d must be an "
                     "orthogonal matrix with determinant of 1! det=" << LinearAlgebra::det(rot.m()));
        }
        cnt++;
        rot.normalize();
    }
    */
    return rot;
}

Rotation2D<> XMLBasisTypes::readRotation2D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, idRotation2D());

    std::vector<double> values = readNVector(element);
    if (values.size() != 4)
        RW_THROW("Expected 4 floating points for Rotation3D. Only "<<values.size()<<" values found");

    return Rotation2D<>(values[0], values[1], values[2], values[3]);
}

Rotation3D<> XMLBasisTypes::readRotation3DStructure(DOMElement* element) {
    if (XMLString::equals(element->getNodeName(), idRotation3D()))
        return readRotation3D(element, false);
    if (XMLString::equals(element->getNodeName(), idRPY()))
        return readRPY(element, false).toRotation3D();
    if (XMLString::equals(element->getNodeName(), idEAA()))
        return readEAA(element, false).toRotation3D();
    if (XMLString::equals(element->getNodeName(), idQuaternion()))
        return readQuaternion(element, false).toRotation3D();

    RW_THROW("Unable to find match \""<<XMLStr(element->getNodeName()).str()<<"\" with (Rotation3D|RPY|EAA|Quaternion)");
    return Rotation3D<>();
}

Transform3D<> XMLBasisTypes::readTransform3D(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, idTransform3D());

    Vector3D<> position(0,0,0);
    Rotation3D<> rotation(Rotation3D<>::identity());
    DOMNodeList* children = element->getChildNodes();
    const  XMLSize_t nodeCount = children->getLength();
    for (XMLSize_t i = 0; i<nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), idMatrix())) {
                std::vector<double> values = readNVector(child);
                if (values.size() != 12)
                    RW_THROW("Expected   <Matrix> with 12 doubles when parsing Transform3D. Found "<<values.size()<<" values");
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
            } else if (XMLString::equals(child->getNodeName(), idPos())) {
                position = readVector3D(child, false);
            } else {
                rotation = readRotation3DStructure(child);
            }

        }
    }
    rotation.normalize();
    return Transform3D<>(position, rotation);
}

VelocityScrew6D<> XMLBasisTypes::readVelocityScrew6D(xercesc::DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, idVelocityScrew6D());

    Vector3D<> linear(0, 0, 0);
    EAA<> angular(0, 0, 0);

    DOMNodeList* children = element->getChildNodes();
    const XMLSize_t nodeCount = children->getLength();
    for (XMLSize_t i = 0; i < nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*> (children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), idLinear())) {
                linear = readVector3D(child, false);
            } else if (XMLString::equals(child->getNodeName(), idAngular())) {
                angular = readEAA(element, false);
            } else {
                RW_THROW("Unknown element \""<<XMLStr(child->getNodeName()).str()<<"\" specified in VelocityScrew6D");
            }
        }
    }

    return VelocityScrew6D<>(linear, angular);

}

std::vector<double> XMLBasisTypes::readDoubleList(xercesc::DOMElement* element, bool doCheckHeader ){
    return readNVector(element);
}

std::vector<int> XMLBasisTypes::readIntList(xercesc::DOMElement* element, bool doCheckHeader ){
    std::vector<double> res2 = readNVector(element);
    std::vector<int> res(res2.size());
    for(size_t i=0;i<res2.size();i++)
        res[i] = (int)res2[i];
    return res;
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
rw::kinematics::State XMLBasisTypes::readState(xercesc::DOMElement* element, WorkCell::Ptr workcell, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, idState());

    State result = workcell->getDefaultState();

    DOMNodeList* children = element->getChildNodes();
    const XMLSize_t nodeCount = children->getLength();
    for (XMLSize_t i = 0; i < nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*> (children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), idQState())) {
                Q q = readQ(child, false);
                if (result.size() != q.size())
                    RW_THROW("Length of State loaded does not match workcell");
                for (size_t i = 0; i<q.size(); i++)
                    result(i) = q(i);

            } else if (XMLString::equals(child->getNodeName(), idTreeState())) {
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
        checkHeader(element, idString());

    return readElementText(element, false);

}

std::vector<std::string> XMLBasisTypes::readStringList(DOMElement* element) {
    DOMNodeList* children = element->getChildNodes();
    const XMLSize_t nodeCount = children->getLength();
    std::vector<std::string> result;
    for (XMLSize_t i = 0; i < nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*> (children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), idString())) {
                std::string str = readString(child, false);
                result.push_back(str);
            }
        }
    }
    return result;
}

XMLBasisTypes::StringPair XMLBasisTypes::readStringPair(DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, idStringPair());

    DOMNodeList* children = element->getChildNodes();
    const XMLSize_t nodeCount = children->getLength();
    std::vector<std::string> result;
    for (XMLSize_t i = 0; i < nodeCount; i++) {
        DOMElement* child = dynamic_cast<DOMElement*> (children->item(i));
        if (child != NULL) {
            if (XMLString::equals(child->getNodeName(), idString())) {
                std::string str = readString(child, false);
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
            if (XMLString::equals(child->getNodeName(), idStringPair())) {
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
        //std::cout<<"readElementText Child = "<<XMLStr(child->getNodeName()).str()<<std::endl;
        if (dynamic_cast<DOMText*>(children->item(0)) != NULL)
            return XMLStr(child->getNodeValue()).str();
    }
    if (exceptionOnEmpty)
        RW_THROW("Unable to find value in Node " + XMLStr(element->getNodeName()).str());
    return "";
}

const XMLCh* XMLBasisTypes::readElementTextXMLCh(xercesc::DOMElement* element, bool exceptionOnEmpty) {
    DOMNodeList* children = element->getChildNodes();

    for (size_t i = 0; i<children->getLength(); i++) {
        DOMNode* child = children->item(i);
        //std::cout<<"readElementText Child = "<<XMLStr(child->getNodeName()).str()<<std::endl;
        if (dynamic_cast<DOMText*>(children->item(0)) != NULL)
            return child->getNodeValue();
            //return XMLStr(child->getNodeValue()).str();
    }
    if (exceptionOnEmpty)
        RW_THROW("Unable to find value in Node " + XMLStr(element->getNodeName()).str());
    return NULL;
}



double XMLBasisTypes::readDouble(xercesc::DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, idDouble());
    double val;
    try {
        val = XMLDouble(readElementTextXMLCh(element)).getValue();
    } catch (...) {
        RW_WARN("Double Value could not be parsed correctly form \""
                << XMLStr(readElementTextXMLCh(element)).str()
                << "\". Setting to NaN! ");
        val = std::numeric_limits<double>::quiet_NaN();
    }
    return val;
    //return std::atof(readElementText(element).c_str());
}

float XMLBasisTypes::readFloat(xercesc::DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, idFloat());
    return (float)XMLDouble(readElementTextXMLCh(element)).getValue();
    //return (float)std::atof(readElementText(element).c_str());
}

int XMLBasisTypes::readInt(xercesc::DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, idInteger());

    return std::atoi(readElementText(element).c_str());
}

bool XMLBasisTypes::readBool(xercesc::DOMElement* element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, idBoolean());

    std::string str = readElementText(element);
    return str == "true";
}


namespace {
    template <class T>
    XMLStr createStringFromArray(const T& v, size_t n) {
        std::ostringstream str;
        str.unsetf(std::ios::floatfield);            // floatfield not set
        str.precision(16);
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

    template <class T>
    XMLStr createStringFromVector(const T& v, size_t n) {
        std::ostringstream str;
        str.unsetf(std::ios::floatfield);            // floatfield not set
        str.precision(16);
        for (size_t i = 0; i<n; ++i) {
            str<<v[i];
            if (i != n-1)
                str<<" ";
        }
        return XMLStr(str.str());
    }

    template <class T>
    XMLStr createStringFromVector(const T& v) {
        return createStringFromVector<T>(v, v.size());
    }

}
    xercesc::DOMElement* XMLBasisTypes::createElement(const XMLCh* id, const XMLCh* value, xercesc::DOMDocument* doc) {
        DOMElement* element = doc->createElement(id);
        DOMText* txt = doc->createTextNode(value);
        element->appendChild(txt);
        return element;
     }



xercesc::DOMElement* XMLBasisTypes::createQ(const Q& q, xercesc::DOMDocument* doc) {
    return createElement(idQ(), createStringFromArray(q).uni(), doc);
    /*DOMElement* element = doc->createElement(QId);
    DOMText* txt = doc->createTextNode(createStringFromArray(q).uni());
    element->appendChild(txt);
    return element;*/
}

xercesc::DOMElement* XMLBasisTypes::createVector3D(const Vector3D<>& v, xercesc::DOMDocument* doc) {
    return createElement(idVector3D(), createStringFromArray(v).uni(), doc);
    /*DOMElement* element = doc->createElement(Vector3DId);

    std::ostringstream str;
    str<<v(0)<<" "<<v(1)<<" "<<v(2);
    DOMText* txt = doc->createTextNode(createVectorString(v).uni());
    element->appendChild(txt);
    return element;*/
}

xercesc::DOMElement* XMLBasisTypes::createVector2D(const rw::math::Vector2D<>& v, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* element = doc->createElement(idVector2D());
    DOMText* txt = doc->createTextNode(createStringFromArray(v).uni());
    element->appendChild(txt);
    return element;
}

xercesc::DOMElement* XMLBasisTypes::createRPY(const rw::math::RPY<>& v, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* element = doc->createElement(idRPY());
    DOMText* txt = doc->createTextNode(createStringFromArray(v, 3).uni());
    element->appendChild(txt);
    return element;
}

xercesc::DOMElement* XMLBasisTypes::createEAA(const rw::math::EAA<>& v, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* element = doc->createElement(idEAA());
    DOMText* txt = doc->createTextNode(createStringFromArray(v, 3).uni());
    element->appendChild(txt);
    return element;
}

xercesc::DOMElement* XMLBasisTypes::createQuaternion(const rw::math::Quaternion<>& q, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* element = doc->createElement(idQuaternion());
    DOMText* txt = doc->createTextNode(createStringFromArray(q, 4).uni());
    element->appendChild(txt);
    return element;
}


DOMElement* XMLBasisTypes::createRotation3D(const rw::math::Rotation3D<>& r, xercesc::DOMDocument* doc) {
    //DOMElement* element = doc->createElement(Rotation3DId);
    std::ostringstream str;
    str.unsetf(std::ios::floatfield);            // floatfield not set
    str.precision(16);
    str<<r(0,0)<<" "<<r(0,1)<<" "<<r(0,2)<<" ";
    str<<r(1,0)<<" "<<r(1,1)<<" "<<r(1,2)<<" ";
    str<<r(2,0)<<" "<<r(2,1)<<" "<<r(2,2);
    return createElement(idRotation3D(), XMLStr(str.str()).uni(), doc);
    /*DOMText* txt = doc->createTextNode(XMLStr(str.str()).uni());
    element->appendChild(txt);
    return element;*/
}

xercesc::DOMElement* XMLBasisTypes::createRotation2D(const rw::math::Rotation2D<>& r, xercesc::DOMDocument* doc) {

    //DOMElement* element = doc->createElement(Rotation2DId);
    std::ostringstream str;
    str.unsetf(std::ios::floatfield);            // floatfield not set
    str.precision(16);
    str<<r(0,0)<<" "<<r(0,1)<<" "<<r(1,0)<<" "<<r(1,1);
    return createElement(idRotation2D(), XMLStr(str.str()).uni(), doc);
    /*DOMText* txt = doc->createTextNode(XMLStr(str.str()).uni());
    element->appendChild(txt);
    return element;*/
}

xercesc::DOMElement* XMLBasisTypes::createTransform3D(const rw::math::Transform3D<>& t, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* element = doc->createElement(idTransform3D());

    xercesc::DOMElement* posElem = createElement(idPos(), createStringFromArray(t.P()).uni(), doc);
    /*DOMElement* posElem = doc->createElement(PosId);
    DOMText* txt = doc->createTextNode(createStringFromArray(t.P()).uni());
    posElem->appendChild(txt);*/


    xercesc::DOMElement* rotElem = createRotation3D(t.R(), doc);

    element->appendChild(posElem);
    element->appendChild(rotElem);

    return element;
}

DOMElement* XMLBasisTypes::createVelocityScrew6D(const rw::math::VelocityScrew6D<>& v, xercesc::DOMDocument* doc) {
    return createElement(idVelocityScrew6D(), createStringFromArray(v, 6).uni(), doc);
    /*DOMElement* element = doc->createElement(VelocityScrew6DId);
    DOMText* txt = doc->createTextNode(createStringFromArray(v, 6).uni());
    element->appendChild(txt);
    return element;*/
}

xercesc::DOMElement* XMLBasisTypes::createIntList(const std::vector<int>& ints, xercesc::DOMDocument* doc){
    return createElement(idIntList(), createStringFromVector(ints).uni(), doc);
}

xercesc::DOMElement* XMLBasisTypes::createDoubleList(const std::vector<double>& doubles, xercesc::DOMDocument* doc){
    return createElement(idDoubleList(), createStringFromVector(doubles).uni(), doc);
}

xercesc::DOMElement* XMLBasisTypes::createQState(const rw::kinematics::State& state, xercesc::DOMDocument* doc) {
    return createElement(idQState(), createStringFromArray<State>(state, state.size()).uni(), doc);

    /*DOMElement* element = doc->createElement(QStateId);
    DOMText* txt = doc->createTextNode(createStringFromArray<State>(state, state.size()).uni());
    element->appendChild(txt);
    return element;*/
}


xercesc::DOMElement* XMLBasisTypes::createBoolean(bool value, xercesc::DOMDocument* doc) {
    return createElement(idBoolean(), XMLStr(value).uni(), doc);
}

xercesc::DOMElement* XMLBasisTypes::createDouble(double value, xercesc::DOMDocument* doc) {
	std::ostringstream str;
	str.unsetf(std::ios::floatfield);            // floatfield not set
	str.precision(16);
	str << value;
    return createElement(idDouble(), XMLStr(str.str()).uni(), doc);
}

xercesc::DOMElement* XMLBasisTypes::createFloat(float value, xercesc::DOMDocument* doc) {
    return createElement(idFloat(), XMLStr(value).uni(), doc);
}


xercesc::DOMElement* XMLBasisTypes::createInteger(int value, xercesc::DOMDocument* doc) {
    return createElement(idInteger(), XMLStr(value).uni(), doc);
}



xercesc::DOMElement* XMLBasisTypes::createString(const std::string& str, xercesc::DOMDocument* doc) {
    return createElement(idString(), XMLStr(str).uni(), doc);
    /*DOMElement* element = doc->createElement(StringId);
    DOMText* txt = doc->createTextNode(XMLStr(str).uni());
    element->appendChild(txt);
    return element;*/
}

xercesc::DOMElement* XMLBasisTypes::createStringList(const std::vector<std::string>& strings, xercesc::DOMDocument* doc){
    xercesc::DOMElement* element = doc->createElement(idStringList());
    BOOST_FOREACH(const std::string& str, strings){
        element->appendChild(createString(str, doc));
    }
    return element;
}


xercesc::DOMElement* XMLBasisTypes::createStringPair(const std::string& first, const std::string& second, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* element = doc->createElement(idStringPair());
    element->appendChild(createString(first, doc));
    element->appendChild(createString(second, doc));
    return element;
}

xercesc::DOMElement* XMLBasisTypes::createTreeState(const rw::kinematics::State& state, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* element = doc->createElement(idTreeState());

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

xercesc::DOMElement* XMLBasisTypes::createState(const rw::kinematics::State& state, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* element = doc->createElement(idState());
    element->appendChild(createQState(state, doc));
    element->appendChild(createTreeState(state, doc));
    return element;
}


