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

#include "XMLPathLoader.hpp"

#include <iostream>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMDocumentType.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMNodeIterator.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMText.hpp>

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>
#include <xercesc/util/XMLDouble.hpp>

#include <xercesc/validators/common/Grammar.hpp>
#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/sax/SAXException.hpp>


#include <rw/math/Q.hpp>
#include <rw/models/WorkCell.hpp>

#include <rw/trajectory/Trajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/CircularInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/LloydHaywardBlend.hpp>

#include "XercesErrorHandler.hpp"
#include "XMLBasisTypes.hpp"
#include "XMLPathFormat.hpp"

using namespace xercesc;
using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::kinematics;
using namespace rw::models;

XMLPathLoader::XMLPathLoader(const std::string& filename, rw::models::WorkCell::Ptr workcell, const std::string& schemaFileName)
{
    _workcell = workcell;

    XercesDOMParser parser;
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, filename, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement();
    readPath(elementRoot);


}


XMLPathLoader::XMLPathLoader(std::istream& instream, rw::models::WorkCell::Ptr workcell, const std::string& schemaFileName) {
    _workcell = workcell;
    XercesDOMParser parser;
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, instream, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement();
    readPath(elementRoot);

}


XMLPathLoader::XMLPathLoader(xercesc::DOMElement* element) {
    readPath(element);
}

XMLPathLoader::~XMLPathLoader()
{
}


namespace {

    template <class T>
    class ElementReader {
    public:
		ElementReader(WorkCell::Ptr workcell = NULL) {
            _workcell = workcell;
        }

        T readElement(xercesc::DOMElement* element);
    protected:
		WorkCell::Ptr _workcell;
    };


    template<> Q ElementReader<Q>::readElement(xercesc::DOMElement* element) {
        return XMLBasisTypes::readQ(element, true);
    }

    template<> Vector3D<> ElementReader<Vector3D<> >::readElement(xercesc::DOMElement* element) {
        return XMLBasisTypes::readVector3D(element, true);
    }

    template<> Rotation3D<> ElementReader<Rotation3D<> >::readElement(xercesc::DOMElement* element) {
        return XMLBasisTypes::readRotation3DStructure(element);
    }


    template<> Transform3D<> ElementReader<Transform3D<> >::readElement(xercesc::DOMElement* element) {
        return XMLBasisTypes::readTransform3D(element, true);
    }

    template<> State ElementReader<State>::readElement(xercesc::DOMElement* element) {
        return XMLBasisTypes::readState(element, _workcell, true);
    }

    template<> TimedQ ElementReader<TimedQ>::readElement(xercesc::DOMElement* element) {
        double time = 0.0;
        Q q;
        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();
        for(XMLSize_t i = 0; i < nodeCount; ++i ) {
            xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
            if (child != NULL) {
                if (XMLString::equals(child->getNodeName(), XMLPathFormat::TimeId)) {
                    time = XMLDouble(child->getChildNodes()->item(0)->getNodeValue()).getValue();
                } else if (XMLString::equals(child->getNodeName(), XMLBasisTypes::QId)) {
                    q = XMLBasisTypes::readQ(child, false);
                }
            }
        }
        return makeTimed(time, q);
    }

    template<> TimedState ElementReader<TimedState>::readElement(xercesc::DOMElement* element) {
        double time = 0.0;
        State state;
        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();
        for(XMLSize_t i = 0; i < nodeCount; ++i ) {
            xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
            if (child != NULL) {
                if (XMLString::equals(child->getNodeName(), XMLPathFormat::TimeId)) {
                    time = XMLDouble(child->getChildNodes()->item(0)->getNodeValue()).getValue();
                } else if (XMLString::equals(child->getNodeName(), XMLBasisTypes::StateId)) {
                    state = XMLBasisTypes::readState(child, _workcell, false);
                }
            }
        }
        return makeTimed(time, state);
    }

    template <class T, class R>
	void read(xercesc::DOMElement* element,R result, WorkCell::Ptr workcell = NULL) {

        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();
        ElementReader<T> reader(workcell);
        //Run through all elements and read in content
        for(XMLSize_t i = 0; i < nodeCount; ++i ) {
            xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
            if (child != NULL) {
                T val = reader.readElement(child);
                result->push_back(val);
            }
        }
    }


} //end namespace




XMLPathLoader::Type XMLPathLoader::getType() {
    return _type;
}


QPath::Ptr XMLPathLoader::getQPath() {
    if (_type != QType)
        RW_THROW("The loaded Path is not of type QPath. Use XMLPathLoader::getType() to read its type");
    return _qPath;
}



Vector3DPath::Ptr XMLPathLoader::getVector3DPath() {
    if (_type != Vector3DType)
        RW_THROW("The loaded Path is not of type Vector3DPath. Use XMLPathLoader::getType() to read its type");
    return _v3dPath;
}

Rotation3DPath::Ptr XMLPathLoader::getRotation3DPath() {
    if (_type != Rotation3DType)
        RW_THROW("The loaded Path is not of type Rotation3DPath. Use XMLPathLoader::getType() to read its type");
    return _r3dPath;
}


Transform3DPath::Ptr XMLPathLoader::getTransform3DPath()
{
    if (_type != Transform3DType)
        RW_THROW("The loaded Path is not of type Transform3DPath. Use XMLPathLoader::getType() to read its type");
    return _t3dPath;
}


StatePath::Ptr XMLPathLoader::getStatePath() {
    if (_type != StateType)
        RW_THROW("The loaded Path is not of type StatePath. Use XMLPathLoader::getType() to read its type");
    return _statePath;
}




rw::trajectory::TimedQPath::Ptr XMLPathLoader::getTimedQPath() {
    if (_type != TimedQType)
        RW_THROW("The loaded Path is not of type TimedQPath. Use XMLPathLoader::getType() to read its type");
    return _timedQPath;
}

rw::trajectory::TimedStatePath::Ptr XMLPathLoader::getTimedStatePath() {
    if (_type != TimedStateType)
        RW_THROW("The loaded Path is not of type TimedStatePath. Use XMLPathLoader::getType() to read its type");
    return _timedStatePath;
}



void XMLPathLoader::readPath(xercesc::DOMElement* element) {
    if (XMLString::equals(XMLPathFormat::QPathId, element->getNodeName())) {
        _qPath = ownedPtr(new QPath());
		read<Q, QPath::Ptr>(element, _qPath);
        _type = QType;
    } else if (XMLString::equals(XMLPathFormat::V3DPathId, element->getNodeName())) {
        _v3dPath = ownedPtr(new Vector3DPath());
		read<Vector3D<>, Vector3DPath::Ptr>(element, _v3dPath);
        _type = Vector3DType;
    } else if (XMLString::equals(XMLPathFormat::R3DPathId, element->getNodeName())) {
        _r3dPath = ownedPtr(new Rotation3DPath());
		read<Rotation3D<>, Rotation3DPath::Ptr>(element, _r3dPath);
        _type = Rotation3DType;
    } else if (XMLString::equals(XMLPathFormat::T3DPathId, element->getNodeName())) {
        _t3dPath = ownedPtr(new Transform3DPath());
		read<Transform3D<>, Transform3DPath::Ptr>(element, _t3dPath);
        _type = Transform3DType;
    } else if(XMLString::equals(XMLPathFormat::StatePathId, element->getNodeName())) {
        _statePath = ownedPtr(new StatePath());
		read<State, StatePath::Ptr>(element, _statePath, _workcell);
        _type = StateType;
    } else if(XMLString::equals(XMLPathFormat::TimedQPathId, element->getNodeName())) {
        _timedQPath = ownedPtr(new TimedQPath());
		read<TimedQ, TimedQPath::Ptr>(element, _timedQPath, _workcell);
        _type = TimedQType;
    } else if(XMLString::equals(XMLPathFormat::TimedStatePathId, element->getNodeName())) {
        _timedStatePath = ownedPtr(new TimedStatePath());
		read<TimedState, TimedStatePath::Ptr>(element, _timedStatePath, _workcell);
        _type = TimedStateType;
    } else {
        //The element is not one we are going to parse.
    }
}

