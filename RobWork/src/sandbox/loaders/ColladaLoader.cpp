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

#include "ColladaLoader.hpp"

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
#include <xercesc/util/XMLStringTokenizer.hpp>
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

#include <rw/loaders/xml/XercesErrorHandler.hpp>
#include <rw/loaders/xml/XMLBasisTypes.hpp>
#include <rw/loaders/xml/XMLPathFormat.hpp>

#include <boost/foreach.hpp>
#include "ColladaFormat.hpp"
#include <rw/loaders/xml/XercesUtils.hpp>

#include <rw/rw.hpp>

using namespace xercesc;
using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::kinematics;
using namespace rw::models;

#define DEBUGL(ostreamExpression) do {       \
    std::cout << ostreamExpression << std::endl; \
} while (0)


ColladaLoader::ColladaLoader(const std::string& filename, const std::string& schemaFileName)
{
    try {
       XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    } catch( XMLException& e ){
       RW_THROW("Xerces initialization Error"<<XMLStr(e.getMessage()).str());
    }

    XercesDOMParser parser;
    DOMDocument* doc = XercesDocumentReader::readDocument(parser, filename, schemaFileName);
    DOMElement* elementRoot = doc->getDocumentElement();

    readColladaWorkCell(elementRoot);
}


ColladaLoader::ColladaLoader(std::istream& instream, const std::string& schemaFileName) {
    try {
       XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    } catch( XMLException& e ){
       RW_THROW("Xerces initialization Error"<<XMLStr(e.getMessage()).str());
    }

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

    class DOMChildVector {
    public:
        DOMChildVector(DOMNodeList* childre):children(childre){};

        struct ChildIterator  {
            /** Iterator category. */
            typedef std::forward_iterator_tag iterator_category;

            /** Value type. */
            typedef DOMNode* value_type;

            /** Pointer type. */
            typedef DOMNode* pointer;

            /** Reference type. */
            typedef DOMNode* reference;

            /** Difference type. */
            typedef ptrdiff_t difference_type;

            ChildIterator(int pos, DOMNodeList* list):_pos(pos),children(list){}
            /**
             * @brief Reference to the T element
             */
            DOMNode* operator*() const {
                current = children->item(_pos);
                return current;
            }

            /**
             * @brief Pointer to the T element
             */
            DOMNode** operator->() const {
                current = children->item(_pos);
                return &current;
            }

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
            mutable DOMNode *current;
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

    class DOMElemChildVector {
    public:
        DOMElemChildVector(DOMElement* parent):_parent(parent){};

        static DOMElemChildVector make(DOMElement* parent){
            return DOMElemChildVector(parent);
        }

        struct ChildIterator  {
            /** Iterator category. */
            typedef std::forward_iterator_tag iterator_category;

            /** Value type. */
            typedef DOMElement* value_type;

            /** Pointer type. */
            typedef DOMElement* pointer;

            /** Reference type. */
            typedef DOMElement* reference;

            /** Difference type. */
            typedef ptrdiff_t difference_type;

            ChildIterator(DOMElement* parent, DOMElement* child):_parent(parent),current(child){}
            /**
             * @brief Reference to the T element
             */
            DOMElement* operator*() const {
                return current;
            }

            /**
             * @brief Pointer to the T element
             */
            DOMElement** operator->() const {
                return &current;
            }

            /**
             * @brief Increments the position of the iterator
             * @return Reference to the incremented iterator
             */
            ChildIterator& operator++()
            {
                if(current!=NULL)
                    current = current->getNextElementSibling();
                return *this;
            }

            /**
             * @brief Increments the position of the iterator
             * @return the ConcatVectorIterator with the value before the incrementation
             */
            ChildIterator operator++(int)
            {
                RW_THROW("not implemented!");
                return *this;
            }

            /**
             * @brief Tests whether the positions of two iterators are equal
             * @param other [in] ConcatVectorIterator to compare with
             * @return true if equal
             */
            bool operator==(const ChildIterator& other) const
            {


                return
                    current == other.current &&
                    _parent == other._parent;
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
                return this->_parent != other._parent || this->current != other.current;
            }
            int _pos;
            mutable DOMElement* _parent, *current;
        };


        typedef ChildIterator iterator;
        typedef const ChildIterator const_iterator;

        iterator begin(){
            return ChildIterator(_parent,_parent->getFirstElementChild());
        }

        iterator end(){
            return ChildIterator(_parent,NULL);
        }

        DOMElement* _parent;
    };


    bool isName(DOMElement* element, const char* str2){
        return XMLString::equals(element->getNodeName(), XMLStr(str2).uni());
    }

    bool isName(DOMElement* element, const std::string str2){
        return XMLString::equals(element->getNodeName(), XMLStr(str2).uni());
    }

    bool isName(DOMElement* element, const XMLCh *str2){
        return XMLString::equals(element->getNodeName(), str2);
    }

    std::string getAttrib(DOMElement* element, const char* str){
        return XMLStr(element->getAttribute(XMLStr(str).uni())).str();
    }

    bool getAttribBoolean(DOMElement* element, const std::string& name, bool defval){
        const XMLCh *val = element->getAttribute(XMLStr(name).uni());
        if( val==NULL)
            return defval;
        if( XMLStr(val).str()=="" )
            return defval;
        if( XMLStr(val).str()=="true"){
            return true;
        }
        return false;
    }

    double getAttribDouble(DOMElement* element, const char* str, double defVal){
        std::cout << str << std::endl;
        const XMLCh *val = element->getAttribute(XMLStr(str).uni());
        if( val==NULL)
            return defVal;
        if( XMLStr(val).str()=="" )
            return defVal;
        double dval = defVal;
        try {
            dval = XMLDouble(val).getValue();
        } catch (...){
            return defVal;
        }
        return dval;
    }
/*
    XMLCh* getAttribute(DOMElement* element, const char* str, const std::string& defval){
        return element->getAttribute(XMLStr(str).uni);
    }
*/

    void getArrayValue(std::string &val, const XMLCh* str){ val = XMLStr(str).str(); }
    void getArrayValue(unsigned int &val, const XMLCh* str){ val = (unsigned int)(XMLDouble(str).getValue()); }
    void getArrayValue(int &val, const XMLCh* str){ val = (int)(XMLDouble(str).getValue()); }
    void getArrayValue(double &val, const XMLCh* str){ val = XMLDouble(str).getValue(); }
    void getArrayValue(bool &val, XMLCh* str){
        if( XMLString::equals(str, XMLStr("true").uni()) ){
            val = true;
        } else {
            val = false;
        }
    }

    template<class T>
    void readNArray(DOMElement *elem, std::vector<T>& array, int n){
        array.resize(n);
        XMLStringTokenizer tokenizer(elem->getNodeValue());
        int i=0;
        T val;
        while(tokenizer.hasMoreTokens()){
            RW_ASSERT_MSG(i<(int)array.size(), i<< "<" << array.size());
            getArrayValue(val, tokenizer.nextToken());
            array[i] = val;
            i++;
        }
    }

    template<class T>
    void readArray(DOMElement *elem, Dae::Array<T>& array){
        array.count = getAttribDouble(elem, "count", 0);
        array.id = getAttrib(elem, "id");
        array.name = getAttrib(elem, "name");
        array.data = ownedPtr<std::vector<T> >( new std::vector<T>(array.count) );

        XMLStringTokenizer tokenizer(elem->getNodeValue());
        int i=0;
        T val;
        while(tokenizer.hasMoreTokens()){
            RW_ASSERT_MSG(i<(int)array.data->size(), i<< "<" << array.data->size());
            getArrayValue(val, tokenizer.nextToken());
            (*array.data)[i] = val;
            i++;
        }
    }
/*
    Dae::TechniqueCommonSource readTechniqueCommonSource(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Source source;
        source.id = getAttrib(element, "id");
        source.name = getAttrib(element, "name");
        DOMChildVector domvect(element->getChildNodes());
        BOOST_FOREACH(DOMNode* node, domvect){
            DOMElement* child = dynamic_cast<DOMElement*>(node);
            if (child == NULL)
                continue;
            std::cout << XMLStr(child->getNodeName()).str() << std::endl;
            if(isName(child,"bool_array")){
            } else if(isName(child,"float_array")){
            } else if(isName(child,"int_array")){
            } else if(isName(child,"technique")){
                RW_WARN("NOT IMPLEMENTED YET!");
            }
        }
        return technique;
    }
*/
    Dae::Param readParam(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Param param;
        param.name = getAttrib(element, "name");
        param.sid = getAttrib(element, "sid");
        param.type = getAttrib(element, "type");
        param.semantic = getAttrib(element, "semantic");
        return param;
    }

    Dae::Accessor readAccessor(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Accessor accessor;
        accessor.count = (int)getAttribDouble(element, "count", 0);
        accessor.offset = (int)getAttribDouble(element, "offset", 0);
        accessor.source = getAttrib(element, "source");
        accessor.stride = (int)getAttribDouble(element, "stride", 1);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if(isName(child,"param")){
                accessor.params.push_back( readParam(child, state) );
            }
        }
        return accessor;
    }

    Dae::Source readSource(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Source source;
        source.id = getAttrib(element, "id");
        source.name = getAttrib(element, "name");

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if(isName(child,"bool_array")){
                readArray(child, source.boolArray);
            } else if(isName(child,"float_array")){
                readArray(child, source.floatArray);
            } else if(isName(child,"int_array")){
                readArray(child, source.intArray);
            } else if(isName(child,"IDREF_array")){
                readArray(child, source.idRefArray);
            } else if(isName(child,"Name_array")){
                readArray(child, source.nameArray);
            } else if(isName(child,"SIDREF_array")){
                readArray(child, source.sidRefArray);
            } else if(isName(child,"token_array")){
                readArray(child, source.tokenArray);
            } else if(isName(child,"technique_common")){
                // it can only have an accessor element, so we parse that straight away
                DOMElement* achild = dynamic_cast<DOMElement*>( child->getElementsByTagName(XMLStr("accessor").uni())->item(0) );
                if(achild!=NULL)
                    source.accessor = readAccessor(achild, state);
            } else if(isName(child,"technique")){
                RW_WARN("NOT IMPLEMENTED YET!");
            }
        }
        return source;
    }

    Dae::Extra readExtra(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Extra extra;

        return extra;
    }

    Dae::Input readInput(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Input input;
        input.semantic = getAttrib(element, "semantic");
        input.source = getAttrib(element, "source");
        return input;
    }

    Dae::InputShared readInputShared(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::InputShared input;
        input.offset = (int)getAttribDouble(element, "offset", 0);
        input.semantic = getAttrib(element, "semantic");
        input.source = getAttrib(element, "source");
        input.set = (int) getAttribDouble(element, "set", 0);
        return input;
    }


    Dae::Vertices readVertices(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Vertices vertices;
        vertices.id = getAttrib(element, "id");
        vertices.name = getAttrib(element, "name");

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if( isName(child, "input") ) {
                vertices.inputs.push_back( readInput(child, state));
            } else if( isName(child, "extra") ){
                vertices.extras.push_back( readExtra(child, state) );
            } else {
                RW_WARN("NOT IMPLEMENTED YET!");
            }
        }
        return vertices;
    }


    Dae::Triangles readTriangles(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Triangles tris;
        tris.name = getAttrib(element, "name");
        tris.count = getAttribDouble(element, "count", 0);
        tris.material = getAttrib(element, "material");

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if ( isName(child, "input" ) ) {
                tris.inputs.push_back( readInputShared(child, state) );
            } else if( isName(child, "p")){
                tris.p = ownedPtr( new std::vector<unsigned int>(tris.count) );
                readNArray(child, *tris.p, tris.count);
            } else if( isName(child, "lines")){
            }
        }


        return tris;
    }

    Dae::Mesh readMesh(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Mesh mesh;
        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if ( isName(child, "source" ) ) {
                mesh.sources.push_back( readSource(child, state) );
            } else if( isName(child, "vertices")){
                mesh.vertices = readVertices(child, state);
            } else if( isName(child, "lines")){
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "linestrips")){
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "polygons")){
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "polylist")){
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "triangles")){
                mesh.tris.push_back( readTriangles(child, state));
            } else if( isName(child, "trifans")){
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "tristrips")){
                /// TODO: implement parsing of all primitives
            } else if( isName(child, "extra")){
                mesh.extras.push_back( readExtra(child,state) );
            } else {
                RW_WARN("NOT IMPLEMENTED YET!");
            }
        }
        return mesh;
    }

    std::string getChildString(DOMElement* element, const std::string& name){
        DEBUGL("getChildString(" << name << ")");
        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );
            if ( isName(child, name ) ) {
                XMLStr xmlstr(child->getNodeValue());
                return xmlstr.str();
            }
        }
        return "";
    }

    Dae::Asset readAsset(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Asset asset;
        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if ( isName(child, "contributor") ) {
                asset.contributer.author = getChildString(child, "author");
                asset.contributer.authoring_tool = getChildString(child, "authoring_tool");
                asset.contributer.comments = getChildString(child, "comments");
            } else if( isName(child, "created" ) ){
                asset.created = XMLStr(child->getNodeValue()).str();
            } else if( isName(child, "modified" ) ){
                asset.modified = XMLStr(child->getNodeValue()).str();
            } else if( isName(child, "unit" ) ){
                asset.unit.meter = XMLDouble(child->getAttribute(XMLStr("meter").uni())).getValue();
                asset.unit.name = XMLStr(child->getAttribute(XMLStr("name").uni())).str();
            } else if( isName(child, "up_axis" ) ){
                asset.up_axis = XMLStr(child->getNodeValue()).str();
            }
        }
        return asset;
    }

    Dae::Material readMaterial(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Material mat;
        mat.id = getAttrib(element, "id");
        mat.name = getAttrib(element, "name");

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if(isName(child, "asset")){
                mat.asset = readAsset(child, state);
            } else if(isName(child, "instance_effect")){
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if(isName(child, "extra")){
                mat.extras.push_back( readExtra(child,state) );
            }
        }

        return mat;
    }

    Dae::InstanceMaterial readInstanceMaterial(DOMElement* element, ColladaLoader::ParserState& state) {
        Dae::InstanceMaterial iMat;
        iMat.url = getAttrib(element, "url");
        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );
            if(isName(child, "technique_override")){
                iMat.ref = getAttrib(child, "ref");
                iMat.pass = getAttrib(child, "pass");
            } else if(isName(child, "bind")){
                Dae::BindFX bind;
                bind.semantic = getAttrib(child, "semantic");
                bind.target = getAttrib(child, "target");
                iMat.binds.push_back( bind );
            } else if(isName(child, "extra")){
                iMat.extras.push_back( readExtra(child,state) );
            }
        }
        return iMat;
    }

    Dae::Render readRender(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Render render;
        render.name = getAttrib(element, "name");
        render.sid = getAttrib(element, "sid");
        render.cameraNode = getAttrib(element, "camera_node");

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );
            if(isName(child, "layer")){
                render.layers.push_back( XMLStr(child->getNodeValue()).str() );
            } else if(isName(child, "instance_material")){
                render.iMaterial = readInstanceMaterial(child, state);
            } else if(isName(child, "extra")){
                render.extras.push_back( readExtra(child,state) );
            }
        }


        return render;
    }

    Dae::EvaluateScene readEvaluateScene(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::EvaluateScene evalScene;
        evalScene.id = getAttrib(element, "id");
        evalScene.name = getAttrib(element, "name");
        evalScene.sid = getAttrib(element, "sid");
        evalScene.enable = getAttribBoolean(element, "enable", true );

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );
            if(isName(child, "asset")){
                evalScene.asset = readAsset(child, state);
            } else if(isName(child, "render")){
                evalScene.renders.push_back( readRender(child, state) );
            } else if(isName(child, "extra")){
                evalScene.extras.push_back( readExtra(child,state) );
            }
        }

        return evalScene;
    }

} //end namespace






Dae::Library<Dae::Camera> ColladaLoader::readLibraryCameras(DOMElement* element, ColladaLoader::ParserState& state) {
    Dae::Library<Dae::Camera> camera;
    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        // TODO parse cameras into camera structure
        RW_WARN("Not implemented yet!");
    }
    return camera;
 }

Dae::Node ColladaLoader::readNode(DOMElement* element, ColladaLoader::ParserState& state){
    Dae::Node node;
    node.id = getAttrib(element, "id");
    node.name = getAttrib(element, "name");
    node.sid = getAttrib(element, "sid");
    node.layers = getAttrib(element, "layer");
    /// TODO: the layers attrib
    std::vector<double> arrayTmp;
    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        Dae::Transform transTmp;
        if(isName(child, "asset")){
            node.asset = readAsset(child, state);
        } else if(isName(child, "lookat")){
            transTmp.sid = getAttrib(child, "sid");
            readNArray(child, arrayTmp, 9);
            transTmp.transform =
                Transform3D<>::makeLookAt( Vector3D<>(arrayTmp[0],arrayTmp[1],arrayTmp[2]),
                                           Vector3D<>(arrayTmp[3],arrayTmp[4],arrayTmp[5]),
                                           Vector3D<>(arrayTmp[6],arrayTmp[7],arrayTmp[8]));
            node.transforms.push_back(transTmp);
        } else if(isName(child, "matrix")){
            transTmp.sid = getAttrib(child, "sid");
            readNArray(child, arrayTmp, 16);
            transTmp.transform =
                Transform3D<>( Vector3D<>(arrayTmp[3],arrayTmp[7],arrayTmp[11]),
                               Rotation3D<>(arrayTmp[0],arrayTmp[1],arrayTmp[2],
                                            arrayTmp[4],arrayTmp[5],arrayTmp[6],
                                            arrayTmp[8],arrayTmp[9],arrayTmp[10]
                                            ) );
            node.transforms.push_back(transTmp);
        } else if(isName(child, "rotate")){
            transTmp.sid = getAttrib(child, "sid");
            readNArray(child, arrayTmp, 4);
            EAA<> eaa(Vector3D<>(arrayTmp[0],arrayTmp[1],arrayTmp[2]),arrayTmp[3]*Deg2Rad);
            transTmp.transform = Transform3D<>( eaa.toRotation3D() );
            node.transforms.push_back(transTmp);
        } else if(isName(child, "scale")){
            transTmp.sid = getAttrib(child, "sid");
            readNArray(child, arrayTmp, 3);
            transTmp.matrix(0,0) = arrayTmp[0];
            transTmp.matrix(1,1) = arrayTmp[1];
            transTmp.matrix(2,2) = arrayTmp[2];
            node.transforms.push_back(transTmp);
        } else if(isName(child, "skew")){
            RW_WARN("NOT IMPLEMENTED!");
        } else if(isName(child, "translate")){
            transTmp.sid = getAttrib(child, "sid");
            readNArray(child, arrayTmp, 3);
            transTmp.transform.P() = Vector3D<>(arrayTmp[0],arrayTmp[1],arrayTmp[2]);
            node.transforms.push_back(transTmp);
        } else if(isName(child, "instance_camera")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "instance_controller")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "instance_geometry")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "instance_light")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "instance_node")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "node")){
            node.nodes.push_back( readNode(child, state) );
        } else if(isName(child, "extra")){
            node.extras.push_back( readExtra(child,state) );
        }
    }

    return node;
}


Dae::Geometry ColladaLoader::readGeometry(DOMElement* element, ColladaLoader::ParserState& state){
    DEBUGL("readGeometry");
    Dae::Geometry geom;
    geom.id = getAttrib(element, "id");
    geom.name = getAttrib(element, "name");

    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if(isName(child, "asset")){
            geom.asset = readAsset(child, state);
        } else if(isName(child, "convex_mesh")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "mesh")){
            readMesh(child, state);
        } else if(isName(child, "spline")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "brep")){
            /// TODO: brep, spline and convex_mesh
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "extra")){
            geom.extras.push_back( readExtra(child,state) );
        }
    }

    return geom;
}

Dae::Library<Dae::Geometry> ColladaLoader::readLibraryGeometries(DOMElement* element, ColladaLoader::ParserState& state) {
    Dae::Library<Dae::Geometry> geomLib;
    geomLib.id = getAttrib(element, "id");
    geomLib.name = getAttrib(element, "name");
    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "asset")) {
            geomLib.asset = readAsset(child, state);
        } else if (isName(child, "geometry")) {
            geomLib.elements.push_back( readGeometry(child, state) );
        } else if (isName(child, "extra")) {
            geomLib.extras.push_back( readExtra(child,state) );
        }
    }
    return geomLib;
}

Dae::VisualScene ColladaLoader::readVisualScene(xercesc::DOMElement* element, ParserState& state){
    Dae::VisualScene scene;
    scene.id = getAttrib(element, "id");
    scene.name = getAttrib(element, "name");
    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "asset")) {
            scene.asset = readAsset(child, state);
        } else if (isName(child, "node")) {
            scene.nodes.push_back( readNode(child, state) );
        } else if (isName(child, "evaluate_scene")) {
            scene.evaluateScenes.push_back( readEvaluateScene(child, state) );
        } else if (isName(child, "extra")) {
            scene.extras.push_back( readExtra(child,state) );
        }
    }
    return scene;
}

Dae::Library<Dae::VisualScene> ColladaLoader::readLibraryVisualScenes(DOMElement* element, ColladaLoader::ParserState& state) {
    Dae::Library<Dae::VisualScene> visualSceneLib;
    visualSceneLib.id = getAttrib(element, "id");
    visualSceneLib.name = getAttrib(element, "name");
    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "asset")) {
            visualSceneLib.asset = readAsset(child, state);
        } else if (isName(child, "visual_scene")) {
            visualSceneLib.elements.push_back( readVisualScene(child, state) );
        } else if (isName(child, "extra")) {
            visualSceneLib.extras.push_back( readExtra(child,state) );
        }
    }
    return visualSceneLib;
}



Dae::Library<Dae::Material> ColladaLoader::readLibraryMaterials(DOMElement* element, ColladaLoader::ParserState& state) {
    Dae::Library<Dae::Material> matLib;
    matLib.id = getAttrib(element, "id");
    matLib.name = getAttrib(element, "name");
    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "asset")) {
            matLib.asset = readAsset(child, state);
        } else if (isName(child, "materials")) {
            matLib.elements.push_back( readMaterial(child, state) );
        } else if (isName(child, "extra")) {
            matLib.extras.push_back( readExtra(child,state) );
        }
    }
    return matLib;
}

void ColladaLoader::readCollada(DOMElement* element, ColladaLoader::ParserState& state) {
    state.data.version = getAttrib(element, "version");
    state.data.xmlns = getAttrib(element, "xmlns");
    state.data.base = getAttrib(element, "base");
    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "asset")) {
            state.data.asset = readAsset(child, state);
        } else if (isName(child, "library_cameras")) {
            readLibraryCameras(child, state);
        } else if (isName(child, "library_lights")) {
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if (isName(child, "library_geometries")) {
            state.data.libGeometries.push_back( readLibraryGeometries(child, state) );
        } else if (isName(child, "library_visual_scenes")) {
            state.data.libVisualScenes.push_back( readLibraryVisualScenes(child, state) );
        } else if (isName(child, "scene")) {
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if (isName(child, "extra")) {
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if (isName(child, "library_materials")) {
            state.data.libMaterials.push_back( readLibraryMaterials(child, state) );
        } else if (isName(child, "library_effects")) {

            RW_WARN("NOT IMPLEMENTED YET!");
        }
    }
}

void ColladaLoader::readColladaWorkCell(DOMElement* element) {
    DEBUGL("readColladaWorkCell");
    ParserState collada;
    if (isName(element, "COLLADA") ) {
        readCollada(element, collada);
    }
}

