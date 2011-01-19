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

#include <rw/trajectory/Trajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/CircularInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/LloydHaywardBlend.hpp>

#include "XercesErrorHandler.hpp"
#include "XMLBasisTypes.hpp"
#include "XMLPathFormat.hpp"

#include <boost/foreach.hpp>
#include "ColladaFormat.hpp"
#include <rw/loaders/xml/XercesUtil.hpp>

using namespace xercesc;
using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::kinematics;
using namespace rw::models;

ColladaLoader::ColladaLoader(const std::string& filename, const std::string& schemaFileName)
{
    XercesDOMParser parser;
    DOMDocument* doc = XercesDocumentReader::readDocument(parser, filename, schemaFileName);
    DOMElement* elementRoot = doc->getDocumentElement();

    readColladaWorkCell(elementRoot);
}


ColladaLoader::ColladaLoader(std::istream& instream, const std::string& schemaFileName) {
    XercesDOMParser parser;
    DOMDocument* doc = XercesDocumentReader::readDocument(parser, instream, schemaFileName);
    DOMElement* elementRoot = doc->getDocumentElement();
    readColladaWorkCell(elementRoot);
}


ColladaLoader::ColladaLoader(DOMElement* element) {
    readColladaWorkCell(element);
}

ColladaLoader::~ColladaLoader()
{
}


namespace {

    template <class T>
    class ElementReader {
    public:
        ElementReader(WorkCellPtr workcell = NULL) {
            _workcell = workcell;
        }

        T readElement(xercesc::DOMElement* element);
    protected:
        WorkCellPtr _workcell;
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

    template<> State ElementReader<State>::readElement(DOMElement* element) {
        return XMLBasisTypes::readState(element, _workcell, true);
    }

    template<> TimedQ ElementReader<TimedQ>::readElement(DOMElement* element) {
        double time;
        Q q;
        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();
        for(XMLSize_t i = 0; i < nodeCount; ++i ) {
            DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
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

    template<> TimedState ElementReader<TimedState>::readElement(DOMElement* element) {
        double time;
        State state;
        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();
        for(XMLSize_t i = 0; i < nodeCount; ++i ) {
            DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
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
    void read(DOMElement* element,R result, WorkCellPtr workcell = NULL) {

        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();
        ElementReader<T> reader(workcell);
        //Run through all elements and read in content
        for(XMLSize_t i = 0; i < nodeCount; ++i ) {
            DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
            if (child != NULL) {
                T val = reader.readElement(child);
                result->push_back(val);
            }
        }
    }

    class DOMChildVector {
    public:
        DOMChildVector(DOMNodeList* children):_children(children){};

        /** Iterator category. */
        typedef std::forward_iterator_tag iterator_category;

        /** Value type. */
        typedef DOMNode value_type;

        /** Pointer type. */
        typedef DOMNode* pointer;

        /** Reference type. */
        typedef DOMNode& reference;

        /** Difference type. */
        typedef ptrdiff_t difference_type;


        struct ChildIterator  {
            ChildIterator(int pos, DOMNodeList* list):_pos(pos),children(list){}
            /**
             * @brief Reference to the T element
             */
            DOMNode& operator*() const { return *children->item(_pos); }

            /**
             * @brief Pointer to the T element
             */
            DOMNode* operator->() const { return children->item(_pos); }

            /**
             * @brief Increments the position of the iterator
             * @return Reference to the incremented iterator
             */
            ChildIterator& operator++()
            {
                _pos++;
                return *this;
            }

            /**
             * @brief Increments the position of the iterator
             * @return the ConcatVectorIterator with the value before the incrementation
             */
            ChildIterator operator++(int)
            {
                ChildIterator before = *this;
                _pos++;
                return before;
            }

            /**
             * @brief Tests whether the positions of two iterators are equal
             * @param other [in] ConcatVectorIterator to compare with
             * @return true if equal
             */
            bool operator==(const ChildIterator& other) const
            {
                return
                    _pos == other._pos &&
                    children == other.children;
            }

            /**
             * @brief Tests whether the positions of two iterators are unequal
             * @param other [in] ConcatVectorIterator to compare with
             * @return true if unequal
             */
            bool operator!=(const ChildIterator& other) const
            {

                // If only comparing pos and other.pos Visual Studio will generate
                // an error message when pos and other.pos does not come from the
                // same vector. A test curr != other.curr has thus been added
                return this->_pos != other._pos || this->children != other.children;
            }
            int _pos;
            DOMNodeList* children;
        };


        typedef ChildIterator iterator;
        typedef const ChildIterator const_iterator;

        iterator begin(){
            return ChildIterator(0,children);
        }

        iterator end(){
            return ChildIterator(children->getLength(),children);
        }

        DOMNodeList* children;
    };


} //end namespace


struct ColladaData {
    struct AssetData {
        struct ContributorData {
            std::string author, authoring_tool, comments;
        } Contributor;
        std::string created, modified;
        struct UnitData{
            double meter;
            std::string name;
        } Unit;
        std::string up_axis;
    } Asset;

};

std::string getChildString(DOMElement* element, const std::string& name){
    BOOST_FOREACH(DOMNode* node, DOMChildVector(element->getChildNodes())){
        DOMElement* child = dynamic_cast<DOMElement*>(node);
        if (child == NULL)
            continue;

        if (XMLString::equals(child->getNodeName(), name)) {
            return XercesUtil::XMLStr(child->getNodeValue()).str();
        }
    }
    return "";
}



 ColladaLoader::readAsset(DOMElement* element, ColladaData& data){
    BOOST_FOREACH(DOMNode* node, DOMChildVector(element->getChildNodes())){
        DOMElement* child = dynamic_cast<DOMElement*>(node);
        if (child == NULL)
            continue;

        if (XMLString::equals(child->getNodeName(), "contributor")) {
            data.Asset.Contributor.author = getChildString(child, "author");
            data.Asset.Contributor.authoring_tool = getChildString(child, "authoring_tool");
            data.Asset.Contributor.comments = getChildString(child, "comments");
        } else if( XMLString::equals(child->getNodeName(), "created" )){
            data.Asset.created = XMLStr(child->getNodeValue()).str();
        } else if( XMLString::equals(child->getNodeName(), "modified" ) ){
            data.Asset.modified = XMLStr(child->getNodeValue()).str();
        } else if( XMLString::equals(child->getNodeName(), "unit"  ) ){
            data.Asset.Unit.meter = XMLDouble(child->getAttribute("meter")).getValue();
            data.Asset.Unit.name = XMLStr(child->getAttribute("name")).str();
        } else if( XMLString::equals(child->getNodeName(), "up_axis"  ) ){
            data.Asset.up_axis = XMLStr(child->getNodeValue()).str();
        }
    }
}

 void ColladaLoader::readLibraryCameras(DOMElement* element, ColladaData& data) {
     BOOST_FOREACH(DOMNode* node, DOMChildVector(element->getChildNodes())){
         DOMElement* child = dynamic_cast<DOMElement*>(node);
         if (child == NULL)
             continue;
         // TODO parse cameras into camera structure
     }
 }

void ColladaLoader::readCollada(DOMElement* element, ColladaData& data) {
    BOOST_FOREACH(DOMNode* node, DOMChildVector(element->getChildNodes())){
        DOMElement* child = dynamic_cast<DOMElement*>(node);
        if (child == NULL)
            continue;

        if (XMLString::equals(child->getNodeName(), ColladaCoreFormat::AssetId)) {
            readAsset(child, data);
        } else if (XMLString::equals(child->getNodeName(), ColladaCoreFormat::LibraryCamerasId)) {
            readLibraryCameras(child, data);
        } else if (XMLString::equals(child->getNodeName(), ColladaCoreFormat::LibraryLightsId)) {

        } else if (XMLString::equals(child->getNodeName(), ColladaCoreFormat::LibraryGeometriesId)) {

        } else if (XMLString::equals(child->getNodeName(), ColladaCoreFormat::LibraryVisualScenesId)) {

        } else if (XMLString::equals(child->getNodeName(), ColladaCoreFormat::SceneId)) {

        } else if (XMLString::equals(child->getNodeName(), ColladaCoreFormat::ExtraId)) {

        } else if (XMLString::equals(child->getNodeName(), ColladaEffectsFormat::LibraryMaterialsId)) {

        } else if (XMLString::equals(child->getNodeName(), ColladaEffectsFormat::LibraryEffectsId)) {

        }
    }
}

void ColladaLoader::readColladaWorkCell(DOMElement* element) {

    if (XMLString::equals(ColladaFormat::COLLADAId, element->getNodeName())) {
        _qPath = ownedPtr(new QPath());
        read<Q, QPathPtr>(element, _qPath);
        _type = QType;
    }
}

