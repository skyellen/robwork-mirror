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
#include <rw/graphics/Model3D.hpp>

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
using namespace rw::geometry;

#define DEBUGL(ostreamExpression) do {       \
    std::cout << ostreamExpression << std::endl; \
} while (0)


ColladaLoader::ColladaLoader(const std::string& schemaFileName):_schemaFileName(schemaFileName)
{
}


ColladaLoader::~ColladaLoader()
{
}


WorkCell::Ptr ColladaLoader::loadWorkCell(const std::string& filename)
{
    try {
       XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    } catch( XMLException& e ){
       RW_THROW("Xerces initialization Error"<<XMLStr(e.getMessage()).str());
    }

    XercesDOMParser parser;
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, filename, _schemaFileName);
    DOMElement* elementRoot = doc->getDocumentElement();

    readColladaWorkCell(elementRoot);
    return _workcell;
}


WorkCell::Ptr ColladaLoader::loadWorkCell(std::istream& instream) {
    try {
       XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    } catch( XMLException& e ){
       RW_THROW("Xerces initialization Error"<<XMLStr(e.getMessage()).str());
    }

    XercesDOMParser parser;
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, instream, _schemaFileName);
    DOMElement* elementRoot = doc->getDocumentElement();
    readColladaWorkCell(elementRoot);
    return _workcell;
}

WorkCell::Ptr ColladaLoader::loadWorkCell(DOMElement* element) {
    readColladaWorkCell(element);
    return _workcell;
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
        //std::cout << "\t" << str << ":";
        const XMLCh *val = element->getAttribute(XMLStr(str).uni());
        double retVal = defVal;
        if( val==NULL){
            retVal = defVal;
        } else if( XMLStr(val).str()=="" ) {
            retVal = defVal;
        } else {
            try {
                retVal = XMLDouble(val).getValue();
            } catch (...){
                //std::cout << "EXCEPTION" << std::endl;
            }
        }
        //std::cout << retVal << std::endl;
        return retVal;
    }
/*
    XMLCh* getAttribute(DOMElement* element, const char* str, const std::string& defval){
        return element->getAttribute(XMLStr(str).uni);
    }
*/
/*
    struct Accessor {

        size_t getStride();

    };
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
        XMLStringTokenizer tokenizer(elem->getTextContent());
        //std::cout << "values: " << XMLStr(elem->getTextContent()).str() << std::endl;
        int i=0;
        T val;
        while(tokenizer.hasMoreTokens()){
            RW_ASSERT_MSG(i<(int)array.size(), i<< "<" << array.size());
            getArrayValue(val, tokenizer.nextToken());
            array[i] = val;
            //std::cout << val << ",";
            i++;
        }
        //std::cout << std::endl;
    }

    template<class T>
    Dae::Array<T>* readArray(DOMElement *elem, ColladaLoader::ParserState& state ){
        Dae::Array<T>* array = state.make<Dae::Array<T> >();
        array->count = getAttribDouble(elem, "count", 0);
        array->id = getAttrib(elem, "id");
        array->name = getAttrib(elem, "name");
        array->data = ownedPtr<std::vector<T> >( new std::vector<T>(array->count) );
        state.add(array, array->id, "");
        //std::cout << "values: " << XMLStr(elem->getTextContent()).str() << std::endl;
        XMLStringTokenizer tokenizer(elem->getTextContent());
        //std::cout << "Count tokens: " << tokenizer.countTokens() << std::endl;
        int i=0;
        T val;
        while(tokenizer.hasMoreTokens()){
            RW_ASSERT_MSG(i<(int)array->data->size(), i<< "<" << array->data->size());
            getArrayValue(val, tokenizer.nextToken());
            (*array->data)[i] = val;
            i++;
            //std::cout << val << ",";
        }
        //std::cout << std::endl;
        return array;
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
    struct ScopedPush{
        ScopedPush(Dae::Data* d, ColladaLoader::ParserState& state):_data(d),_state(state){
            if(d->id=="" && d->sid=="")
                return;
            _state.push(d);
        }
        ~ScopedPush(){
            if(_data->id=="" && _data->sid=="")
                return;
            _state.pop();
        }
        Dae::Data *_data;
        ColladaLoader::ParserState& _state;
    };


    size_t getStride(std::vector<Dae::InputShared*>& inputs){
        size_t stride = 0;
        BOOST_FOREACH(Dae::InputShared* input,  inputs){
            if(input->semantic=="TEXCOORD"){
                stride+=1;
            } else if(input->semantic=="VERTEX"){
                stride+=1;
            } else {
                RW_THROW("Unknown semantic!");
            }

        }
        return stride;
    }




    Dae::Param* readParam(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Param *param = state.make<Dae::Param>();
        param->name = getAttrib(element, "name");
        std::cout << "parname: " << param->name << std::endl;
        param->sid = getAttrib(element, "sid");
        param->type = getAttrib(element, "type");
        param->semantic = getAttrib(element, "semantic");
        state.add(param, "", param->sid);
        return param;
    }

    Dae::Accessor* readAccessor(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Accessor *accessor = state.make<Dae::Accessor>();
        accessor->count = (int)getAttribDouble(element, "count", 0);
        accessor->offset = (int)getAttribDouble(element, "offset", 0);
        accessor->source = getAttrib(element, "source");
        accessor->stride = (int)getAttribDouble(element, "stride", 1);
        ScopedPush p(accessor, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if(isName(child,"param")){
                accessor->params.push_back( readParam(child, state) );
            }
        }
        std::cout << "Accessor params: " << accessor->params.size() << std::endl;
        return accessor;
    }

    Dae::Source* readSource(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Source *source = state.make<Dae::Source>();
        source->id = getAttrib(element, "id");
        source->name = getAttrib(element, "name");
        state.add(source, source->id,"");
        ScopedPush p(source, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if(isName(child,"bool_array")){
                source->boolArray = readArray<bool>(child, state);
            } else if(isName(child,"float_array")){
                source->floatArray = readArray<double>(child, state);
            } else if(isName(child,"int_array")){
                source->intArray = readArray<int>(child, state);
            } else if(isName(child,"IDREF_array")){
                source->idRefArray = readArray<std::string>(child, state);
            } else if(isName(child,"Name_array")){
                source->nameArray = readArray<std::string>(child, state);
            } else if(isName(child,"SIDREF_array")){
                source->sidRefArray = readArray<std::string>(child, state );
            } else if(isName(child,"token_array")){
                source->tokenArray = readArray<std::string>(child, state);
            } else if(isName(child,"technique_common")){
                // it can only have an accessor element, so we parse that straight away
                DOMElement* achild = dynamic_cast<DOMElement*>( child->getElementsByTagName(XMLStr("accessor").uni())->item(0) );
                if(achild!=NULL)
                    source->accessor = readAccessor(achild, state);
            } else if(isName(child,"technique")){
                RW_WARN("NOT IMPLEMENTED YET!");
            }
        }
        return source;
    }

    Dae::Extra* readExtra(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Extra *extra = state.make<Dae::Extra>();

        return extra;
    }

    Dae::Input* readInput(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Input *input = state.make<Dae::Input>();
        input->semantic = getAttrib(element, "semantic");
        input->source = getAttrib(element, "source");
        return input;
    }

    Dae::InputShared* readInputShared(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::InputShared *input = state.make<Dae::InputShared>();
        input->offset = (int)getAttribDouble(element, "offset", 0);
        input->semantic = getAttrib(element, "semantic");
        input->source = getAttrib(element, "source");
        input->set = (int) getAttribDouble(element, "set", 0);
        return input;
    }


    Dae::Vertices* readVertices(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Vertices *vertices = state.make<Dae::Vertices>();
        vertices->id = getAttrib(element, "id");
        vertices->name = getAttrib(element, "name");
        state.add(vertices, vertices->id, "");
        ScopedPush p(vertices,state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if( isName(child, "input") ) {
                vertices->inputs.push_back( readInput(child, state));
            } else if( isName(child, "extra") ){
                vertices->extras.push_back( readExtra(child, state) );
            } else {
                RW_WARN("NOT IMPLEMENTED YET!");
            }
        }
        return vertices;
    }


    Dae::Triangles* readTriangles(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Triangles *tris = state.make<Dae::Triangles>();
        tris->name = getAttrib(element, "name");
        tris->count = getAttribDouble(element, "count", 0);
        tris->material = getAttrib(element, "material");
        ScopedPush p(tris, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if ( isName(child, "input" ) ) {
                tris->inputs.push_back( readInputShared(child, state) );
            } else if( isName(child, "p")){
                // calculate the stride based on the inputs
                size_t tristride = getStride(tris->inputs)*3;// the getStride is per vertex
                tris->p = ownedPtr( new std::vector<unsigned int>( tristride*tris->count ) );
                readNArray(child, *tris->p, tristride*tris->count);
            } else if( isName(child, "extra")){

            }
        }


        return tris;
    }

    Dae::Mesh* readMesh(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Mesh* mesh = state.make<Dae::Mesh>();

        ScopedPush p(mesh, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if ( isName(child, "source" ) ) {
                mesh->sources.push_back( readSource(child, state) );
            } else if( isName(child, "vertices")){
                mesh->vertices = readVertices(child, state);
            } else if( isName(child, "lines")){
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "linestrips")){
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "polygons")){
                //mesh->.push_back( readPolygons(child, state));
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "polylist")){
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "triangles")){
                mesh->tris.push_back( readTriangles(child, state));
            } else if( isName(child, "trifans")){
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "tristrips")){
                /// TODO: implement parsing of all primitives
                RW_WARN("NOT IMPLEMENTED YET!");
            } else if( isName(child, "extra")){
                mesh->extras.push_back( readExtra(child,state) );
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
                XMLStr xmlstr(child->getTextContent());
                return xmlstr.str();
            }
        }
        return "";
    }

    Dae::Asset* readAsset(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Asset *asset = state.make<Dae::Asset>();
        ScopedPush p(asset, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if ( isName(child, "contributor") ) {
                //asset->contributer = state.make<Dae::Contributor>();
                asset->contributer.author = getChildString(child, "author");
                asset->contributer.authoring_tool = getChildString(child, "authoring_tool");
                asset->contributer.comments = getChildString(child, "comments");
            } else if( isName(child, "created" ) ){
                asset->created = XMLStr(child->getTextContent()).str();
            } else if( isName(child, "modified" ) ){
                asset->modified = XMLStr(child->getTextContent()).str();
            } else if( isName(child, "unit" ) ){
                //asset->unit = state.make<Dae::Unit>();
                asset->unit.meter = XMLDouble(child->getAttribute(XMLStr("meter").uni())).getValue();
                asset->unit.name = XMLStr(child->getAttribute(XMLStr("name").uni())).str();
            } else if( isName(child, "up_axis" ) ){
                asset->up_axis = XMLStr(child->getTextContent()).str();
            }
        }
        return asset;
    }

    Dae::InstanceEffect* readInstanceEffect(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::InstanceEffect* mat = state.make<Dae::InstanceEffect>();
        mat->sid = getAttrib(element, "sid");
        mat->name = getAttrib(element, "name");
        mat->url = getAttrib(element, "url");
        state.add(mat, "", mat->sid);
        ScopedPush p(mat, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );
            if(isName(child, "technique_hint")){
                RW_WARN("NOT IMPL YET");
            } else if(isName(child, "setparam")){
                RW_WARN("NOT IMPL YET");
            } else if(isName(child, "extra")){
                mat->extras.push_back( readExtra(child,state) );
            }
        }

        return mat;

    }

    Dae::Material* readMaterial(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Material* mat = state.make<Dae::Material>();
        mat->id = getAttrib(element, "id");
        mat->name = getAttrib(element, "name");
        state.add(mat, mat->id,"");
        ScopedPush p(mat, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );

            if(isName(child, "asset")){
                mat->asset = readAsset(child, state);
            } else if(isName(child, "instance_effect")){
                mat->iEffect = readInstanceEffect(child, state);
            } else if(isName(child, "extra")){
                mat->extras.push_back( readExtra(child,state) );
            }
        }

        return mat;
    }

    Dae::InstanceMaterial* readInstanceMaterial(DOMElement* element, ColladaLoader::ParserState& state) {
        Dae::InstanceMaterial* iMat = state.make<Dae::InstanceMaterial>();
        iMat->url = getAttrib(element, "url");
        ScopedPush p(iMat, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );
            if(isName(child, "technique_override")){
                iMat->ref = getAttrib(child, "ref");
                iMat->pass = getAttrib(child, "pass");
            } else if(isName(child, "bind")){
                //Dae::BindFX bind;
                //bind.semantic = getAttrib(child, "semantic");
                //bind.target = getAttrib(child, "target");
                //iMat.binds.push_back( bind );
            } else if(isName(child, "extra")){
                iMat->extras.push_back( readExtra(child,state) );
            }
        }
        return iMat;
    }

    Dae::Render* readRender(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Render *render = state.make<Dae::Render>();
        render->name = getAttrib(element, "name");
        render->sid = getAttrib(element, "sid");
        render->cameraNode = getAttrib(element, "camera_node");
        state.add(render, "", render->sid);
        ScopedPush p(render, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );
            if(isName(child, "layer")){
                render->layers.push_back( XMLStr(child->getTextContent()).str() );
            } else if(isName(child, "instance_material")){
                render->iMaterial = readInstanceMaterial(child, state);
            } else if(isName(child, "extra")){
                render->extras.push_back( readExtra(child,state) );
            }
        }

        return render;
    }

    Dae::EvaluateScene* readEvaluateScene(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::EvaluateScene *evalScene = state.make<Dae::EvaluateScene>();
        evalScene->id = getAttrib(element, "id");
        evalScene->name = getAttrib(element, "name");
        evalScene->sid = getAttrib(element, "sid");
        evalScene->enable = getAttribBoolean(element, "enable", true );
        state.add(evalScene, evalScene->id, evalScene->sid);
        ScopedPush p(evalScene, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );
            if(isName(child, "asset")){
                evalScene->asset = readAsset(child, state);
            } else if(isName(child, "render")){
                evalScene->renders.push_back( readRender(child, state) );
            } else if(isName(child, "extra")){
                evalScene->extras.push_back( readExtra(child,state) );
            }
        }

        return evalScene;
    }


    Dae::Effect* readEffect(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::Effect *effect = state.make<Dae::Effect>();
        effect->id = getAttrib(element, "id");
        effect->name = getAttrib(element, "name");
        state.add(effect, effect->id, effect->sid);
        ScopedPush p(effect, state);

        DOMElemChildVector domChildren(element);
        BOOST_FOREACH(DOMElement* child, domChildren){
            DEBUGL( XMLStr(child->getNodeName()).str() );
            if(isName(child, "asset")){
                effect->asset = readAsset(child, state);
            } else if(isName(child, "newparam")){
                RW_WARN("");
            } else if(isName(child, "extra")){
                effect->extras.push_back( readExtra(child,state) );
            }
        }
        return effect;
    }


    Dae::InstanceNode* readInstanceNode(DOMElement* element, ColladaLoader::ParserState& state){
        Dae::InstanceNode *node = state.make<Dae::InstanceNode>();
        node->sid = getAttrib(element, "sid");
        node->name = getAttrib(element, "name");
        node->url = getAttrib(element, "url");
        node->proxy = getAttrib(element, "proxy");
        state.add(node, "", node->sid);
        ScopedPush p(node, state);
        return node;
    }

    struct Accessor {
        Accessor():_acc(NULL){}
        Accessor(Dae::Accessor* accessor):_acc(accessor)
        {
            //std::cout << "Nr params: " << _acc->params.size() << std::endl;
            //std::cout << _acc->stride << std::endl;
            //std::cout << _acc->offset << std::endl;
            //std::cout << _acc->name << std::endl;
        }

        int getVal(int i){
            return _acc->offset + i*_acc->stride;
        }

        int getValIndex(const std::string& v){

            int i=0;
            BOOST_FOREACH(Dae::Param *param, _acc->params){
                if(param->name==v)
                    return i;
                i++;
            }
            RW_THROW("No such param in accessor! " << v);
            return -1;
        }

        int getDOF(){
            if(_acc==NULL)
                return 0;
            return _acc->params.size();
        }

        size_t size(){
            if(_acc==NULL)
                return 0;
            return _acc->count;
        };

        Dae::Accessor *_acc;
    };

    std::vector<std::string> getPosParams(){
        std::vector<std::string> params;
        params.push_back("X");
        params.push_back("Y");
        params.push_back("Z");
        return params;
    }

    std::vector<std::string> getTexParams(){
        std::vector<std::string> params;
        params.push_back("S");
        params.push_back("T");
        return params;
    }

    struct AccessorFloat {
        AccessorFloat(Dae::Source* source, std::vector<std::string> params):
            _source(source),_params(params),_param_offsets(params.size(),0),_floats(params.size(),0)
        {
            if(source!=NULL){
                // the pos accessor must be have a float array as source
                RW_ASSERT(_source->floatArray!=NULL);
                _accessor = Accessor(source->accessor);
                for(size_t i=0;i<params.size();i++){
                    _param_offsets[i] = _accessor.getValIndex(params[i]);
                }
            }
        }

        const std::vector<double>& get(int i){
            std::vector<double> &fdata = *_source->floatArray->data;
            int offset = _accessor.getVal(i);
            for(size_t j=0;j<_floats.size();j++)
                _floats[j] = fdata[ offset+_param_offsets[j] ];
            return _floats;
        }

        template<class ARRAY_TYPE>
        void get(int i, ARRAY_TYPE& dst){
            std::vector<double> &fdata = *_source->floatArray->data;
            int offset = _accessor.getVal(i);
            for(int j=0;j<_floats.size();j++)
                dst[j] = fdata[ offset+_param_offsets[j] ];
        }

        size_t size(){
            if(_source==NULL)
                return 0;
            return _accessor.size();
        };

        Dae::Source* _source;
        Accessor _accessor;
        std::vector<std::string> _params;
        std::vector<int> _param_offsets;
        std::vector<double> _floats;
    };


    /**
     * @brief Acessor for position sources that is X, Y, Z coordinate sources
     */
    struct PosAccessor {
        PosAccessor(Dae::Source* source): _source(source)
        {
            // the pos accessor must be have a float array as source
            if(source!=NULL){
                RW_ASSERT(_source->floatArray!=NULL);

                _accessor = Accessor(source->accessor);
                // we need to know the indexes of the X Y and Z coordinate
                _X = _accessor.getValIndex("X");
                _Y = _accessor.getValIndex("Y");
                _Z = _accessor.getValIndex("Z");
            }
        }

        Vector3D<> get(int i){
            Vector3D<> res;
            std::vector<double> &fdata = *_source->floatArray->data;
            int offset = _accessor.getVal(i);
            res[0] = fdata[ offset+_X ];
            res[1] = fdata[ offset+_Y ];
            res[2] = fdata[ offset+_Z ];
            return res;
        }

        size_t size(){ return _accessor.size();};

        Dae::Source* _source;
        Accessor _accessor;
        int _X, _Y, _Z;
    };

    /**
     * @brief used to access specific elements in a vector of indices.
     */
    struct TriIndiciesAccessor {
        TriIndiciesAccessor(std::vector<Dae::InputShared*>& inputs_tmp):inputs(inputs_tmp)
        {
            // we know that the stride is at least 3
            offsets = std::vector<int>(10, 0);
            stride_per_vertex = 0;
            vertOffset = 0;
            BOOST_FOREACH(Dae::InputShared* input, inputs){
                if(input->semantic == "VERTEX"){
                    vertOffset = input->offset;
                    if(offsets[input->offset]<1){
                        stride_per_vertex += 1;
                        offsets[input->offset]=1;
                    }
                } else if(input->semantic == "TEXCOORD"){
                    // two texcoords are represented in
                    if(offsets[input->offset]<1){
                        stride_per_vertex -= offsets[input->offset];
                        stride_per_vertex += 1;
                        offsets[input->offset]=1;
                    }
                } else {
                    RW_WARN("unsupported semantic!");
                }
            }

            std::cout << "STRIDE PER VERTEX: " << stride_per_vertex << "  " << vertOffset << std::endl;
        }
        int getIdx(int i){
            return i*stride_per_vertex;
        }
        int getVertexIdx(int i){
            return i*stride_per_vertex+vertOffset;
        }

        int getOffset(std::string semantic){
            BOOST_FOREACH(Dae::InputShared* input, inputs){
                if(input->semantic==semantic)
                    return input->offset;
            }
            return -1;
        }

        std::vector<int> offsets;
        size_t stride_per_vertex, vertOffset;
        std::vector<Dae::InputShared*> inputs;
    };


    Dae::Source* getSource(const std::string& semantic, std::vector<Dae::Input*>& inputs, ColladaLoader::ParserState& state){
        BOOST_FOREACH(Dae::Input* input,  inputs){
            if(semantic==input->semantic)
                return state.get<Dae::Source>(input->source);
        }
        return NULL;
    }


    bool has(const std::string& semantic, std::vector<Dae::Input*>& inputs){
        BOOST_FOREACH(Dae::Input* input,  inputs){
            if(semantic==input->semantic)
                return true;
        }
        return false;
    }


    using namespace rw::graphics;
    rw::graphics::Model3D::Ptr makeRWMesh(Dae::Mesh *mesh, ColladaLoader::ParserState& state){
        std::cout << "makeRWMesh" << std::endl;
        rw::graphics::Model3D::Ptr model = ownedPtr( new rw::graphics::Model3D() );

        std::vector<TriMesh::Ptr> rwmeshes;

        // for now only Triangle meshes are supported

        // we ignore source because its merilly a data container

        //
        //Dae::Vertices *verts = mesh->vertices;


        BOOST_FOREACH(Dae::Triangles* tri, mesh->tris){
            std::string triname = tri->name;
            if(triname=="")
                triname = tri->id;
            std::string material = tri->material;
            Dae::Material* matdata = state.get<Dae::Material>(material);
            //Dae::Effect* effect = state.get<Dae::Effect>(matdata->iEffect->url);

            std::cout << triname << " " << material << std::endl;
            // each vertice points to some source

            Dae::Vertices *triVertex = NULL;
            Dae::Source *posSource = NULL;
            Dae::Source *normSource = NULL;
            Dae::Source *texSource = NULL;
            // the stride indicates how many inputs there are
            int stride = tri->inputs.size();

            BOOST_FOREACH(Dae::InputShared *ishared, tri->inputs){
                // here we just check that we can find the source
                if(ishared->semantic=="VERTEX"){
                    triVertex = state.get<Dae::Vertices>(ishared->source);
                    RW_ASSERT(triVertex);
                } else if( ishared->semantic=="POSITION" ){
                    posSource = state.get<Dae::Source>(ishared->source);
                } else if( ishared->semantic=="NORMAL" ){
                    normSource = state.get<Dae::Source>(ishared->source);
                } else if( ishared->semantic=="TEXCOORD" ){
                    texSource  = state.get<Dae::Source>(ishared->source);
                } else {
                    RW_THROW("type of source not impl");
                    Dae::Source* source = state.get<Dae::Source>(ishared->source);
                    RW_ASSERT(source);
                }
            }
            if(stride == 0){
                // there are no inputs... So we default to VERTEX input
                RW_THROW("What to do, no inputs...");
            }
            if( triVertex!=NULL ){
                posSource = getSource("POSITION", triVertex->inputs, state);
                normSource = getSource("NORMAL", triVertex->inputs, state);
            }
            if(posSource)
                std::cout << "POSITION SOURCE: " << posSource->id << std::endl;
            if(normSource)
                std::cout << "NORMAL SOURCE  : " << normSource->id << std::endl;
            if(texSource)
                std::cout << "TEXCOORD SOURCE  : " << texSource->id << std::endl;

            // create accessor
            AccessorFloat pacc( posSource, getPosParams() );
            AccessorFloat nacc( normSource, getPosParams() );
            AccessorFloat texacc( texSource, getTexParams() );

            Dae::Asset* asset = state.getAsset( posSource );

            // construct the containers for the triangles, TODO: use the most efficient container
            std::vector<Vector3D<float> > *rwvertices = new std::vector<Vector3D<float> >( pacc.size() );
            std::vector<Vector3D<float> > *rwnormals = new std::vector<Vector3D<float> >( nacc.size() );
            std::vector<Vector2D<float> > *rwtex = new std::vector<Vector2D<float> >( texacc.size() );

            IndexedTriMeshN0<float, uint16_t>::TriangleArray *rwtris = new IndexedTriMeshN0<float, uint16_t>::TriangleArray();

            Vector2D<> pos2d;
            Vector3D<> pos;
            for(size_t i=0;i<pacc.size();i++){
                pacc.get(i, pos);
                (*rwvertices)[i] = cast<float>( (pos*asset->unit.meter) );
            }

            for(size_t i=0;i<nacc.size();i++){
                 nacc.get(i, pos);
                (*rwnormals)[i] = cast<float>( pos );
            }

            for(size_t i=0;i<texacc.size();i++){
                 texacc.get(i, pos2d);
                (*rwtex)[i] = cast<float>( pos2d );
            }

            TriIndiciesAccessor iacc( tri->inputs );
            std::vector<unsigned int> &indices = *tri->p;
            for(size_t i=0;i<tri->count*3;i+=3){
                // three indicies compose one triangle
                // we need to read Vertices and optionally, normals and textures
                rwtris->push_back( IndexedTriangle<uint16_t>(indices[ iacc.getVertexIdx(i+0) ],
                                                             indices[ iacc.getVertexIdx(i+1) ],
                                                             indices[ iacc.getVertexIdx(i+2) ] ) );

                //std::cout << "Tri: " << indices[i+0] << " " << indices[i+1] << " " <<  indices[i+2] << std::endl;
            }

            // add the parsed material to Model3D

            //int matid = model->addMaterial();

            // create the object
            Model3D::Object3D::Ptr obj = ownedPtr(new Model3D::Object3D(triname));
            //obj->setMaterial();

            IndexedTriMeshN0<float, uint16_t>::Ptr mymesh = ownedPtr(new IndexedTriMeshN0<float, uint16_t>(rwvertices, rwnormals, rwtris));
            rwmeshes.push_back( mymesh );
        }

        return model;
    }

    std::vector<Model3D::Ptr> makeRWGeometry(Dae::Geometry *geom, ColladaLoader::ParserState& state){
        std::cout << "RWGEom" << std::endl;
        std::vector<Model3D::Ptr> geoms;
        RW_ASSERT(geom);
        std::string name = geom->name;
        std::cout << "Geom name: "<< name << std::endl;
        BOOST_FOREACH(Dae::Mesh* mesh, geom->meshes){
            std::cout << "mesh" << std::endl;
            Model3D::Ptr rwmesh = makeRWMesh(mesh, state);
            //for(size_t i=0;i<rwmesh.size();i++){
            //    geoms.push_back( ownedPtr( new Geometry(rwmesh[i]) ));
            //}
            geoms.push_back( rwmesh );
        }

        std::cout << "RWGEom_end" << std::endl;
        return geoms;
    }


} //end namespace






Dae::Library<Dae::Camera>* ColladaLoader::readLibraryCameras(DOMElement* element, ColladaLoader::ParserState& state) {
    Dae::Library<Dae::Camera>* camera = state.make<Dae::Library<Dae::Camera> >();
    ScopedPush p(camera, state);

    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        // TODO parse cameras into camera structure
        RW_WARN("Not implemented yet!");
    }
    return camera;
 }

Dae::Node* ColladaLoader::readNode(DOMElement* element, ColladaLoader::ParserState& state){
    Dae::Node *node = state.make<Dae::Node>();
    node->id = getAttrib(element, "id");
    node->name = getAttrib(element, "name");
    node->sid = getAttrib(element, "sid");
    node->layers = getAttrib(element, "layer");
    state.add(node, node->id, node->sid);
    ScopedPush p(node, state);

    /// TODO: the layers attrib
    std::vector<double> arrayTmp;
    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );

        if(isName(child, "asset")){
            node->asset = readAsset(child, state);
        } else if(isName(child, "lookat")){
            Dae::Transform *transTmp = state.make<Dae::Transform>();
            transTmp->sid = getAttrib(child, "sid");
            state.add(transTmp, "", transTmp->sid);
            readNArray(child, arrayTmp, 9);
            transTmp->transform =
                Transform3D<>::makeLookAt( Vector3D<>(arrayTmp[0],arrayTmp[1],arrayTmp[2]),
                                           Vector3D<>(arrayTmp[3],arrayTmp[4],arrayTmp[5]),
                                           Vector3D<>(arrayTmp[6],arrayTmp[7],arrayTmp[8]));
            node->transforms.push_back(transTmp);
        } else if(isName(child, "matrix")){
            Dae::Transform *transTmp = state.make<Dae::Transform>();
            transTmp->sid = getAttrib(child, "sid");
            state.add(transTmp, "", transTmp->sid);
            ScopedPush p(transTmp, state);
            readNArray(child, arrayTmp, 16);
            transTmp->transform =
                Transform3D<>( Vector3D<>(arrayTmp[3],arrayTmp[7],arrayTmp[11]),
                               Rotation3D<>(arrayTmp[0],arrayTmp[1],arrayTmp[2],
                                            arrayTmp[4],arrayTmp[5],arrayTmp[6],
                                            arrayTmp[8],arrayTmp[9],arrayTmp[10]
                                            ) );
            node->transforms.push_back(transTmp);
        } else if(isName(child, "rotate")){
            Dae::Transform *transTmp = state.make<Dae::Transform>();
            transTmp->sid = getAttrib(child, "sid");
            state.add(transTmp, "", transTmp->sid);
            ScopedPush p(transTmp, state);
            readNArray(child, arrayTmp, 4);
            EAA<> eaa(Vector3D<>(arrayTmp[0],arrayTmp[1],arrayTmp[2]),arrayTmp[3]*Deg2Rad);
            transTmp->transform = Transform3D<>( eaa.toRotation3D() );
            node->transforms.push_back(transTmp);
        } else if(isName(child, "scale")){
            Dae::Transform *transTmp = state.make<Dae::Transform>();
            transTmp->sid = getAttrib(child, "sid");
            state.add(transTmp, "", transTmp->sid);
            ScopedPush p(transTmp, state);
            readNArray(child, arrayTmp, 3);
            transTmp->matrix(0,0) = arrayTmp[0];
            transTmp->matrix(1,1) = arrayTmp[1];
            transTmp->matrix(2,2) = arrayTmp[2];
            node->transforms.push_back(transTmp);
        } else if(isName(child, "skew")){
            RW_WARN("NOT IMPLEMENTED!");
        } else if(isName(child, "translate")){
            Dae::Transform *transTmp = state.make<Dae::Transform>();
            transTmp->sid = getAttrib(child, "sid");
            state.add(transTmp, "", transTmp->sid);
            ScopedPush p(transTmp, state);
            readNArray(child, arrayTmp, 3);
            transTmp->transform.P() = Vector3D<>(arrayTmp[0],arrayTmp[1],arrayTmp[2]);
            node->transforms.push_back(transTmp);
        } else if(isName(child, "instance_camera")){
            // create an instance of a geometry
            Dae::InstanceCamera *icam = state.make<Dae::InstanceCamera>();
            icam->sid = getAttrib(child, "sid");
            icam->name = getAttrib(child, "name");
            icam->url = getAttrib(child, "url");
            state.add(icam, "", icam->sid);
            ScopedPush p(icam, state);
            node->icameras.push_back(icam);
        } else if(isName(child, "instance_controller")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "instance_geometry")){
            // create an instance of a geometry
            Dae::InstanceGeometry *igeom = state.make<Dae::InstanceGeometry>();
            igeom->sid = getAttrib(child, "sid");
            igeom->name = getAttrib(child, "name");
            igeom->url = getAttrib(child, "url");
            std::cout << "URL: " << igeom->url << std::endl;
            state.add(igeom, "", igeom->sid);
            ScopedPush p(igeom, state);
            //state.get<Dae::Geometry>(url);
            //Dae::Geometry* geom = state.data.get<Dae::Geometry>(url);
            node->igeometries.push_back(igeom);
        } else if(isName(child, "instance_light")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "instance_node")){
            node->inodes.push_back( readInstanceNode(child, state));
        } else if(isName(child, "node")){
            node->nodes.push_back( readNode(child, state) );
        } else if(isName(child, "extra")){
            node->extras.push_back( readExtra(child,state) );
        }
    }

    return node;
}


Dae::Geometry* ColladaLoader::readGeometry(DOMElement* element, ColladaLoader::ParserState& state){
    DEBUGL("readGeometry");
    Dae::Geometry *geom = state.make<Dae::Geometry>();
    geom->id = getAttrib(element, "id");
    geom->name = getAttrib(element, "name");
    std::cout << "Attrib ID: " << geom->id << std::endl;
    state.add(geom, geom->id, "");
    ScopedPush p(geom, state);

    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if(isName(child, "asset")){
            geom->asset = readAsset(child, state);
        } else if(isName(child, "convex_mesh")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "mesh")){
            // TODO: do something with mesh
            geom->meshes.push_back( readMesh(child, state) );
        } else if(isName(child, "spline")){
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "brep")){
            /// TODO: brep, spline and convex_mesh
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if(isName(child, "extra")){
            geom->extras.push_back( readExtra(child,state) );
        }
    }

    return geom;
}

Dae::Library<Dae::Geometry>* ColladaLoader::readLibraryGeometries(DOMElement* element, ColladaLoader::ParserState& state) {
    Dae::Library<Dae::Geometry> *geomLib = state.make< Dae::Library<Dae::Geometry> >();
    geomLib->id = getAttrib(element, "id");
    geomLib->name = getAttrib(element, "name");
    state.add(geomLib, geomLib->id, "");
    ScopedPush p(geomLib, state);

    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "asset")) {
            geomLib->asset = readAsset(child, state);
        } else if (isName(child, "geometry")) {
            geomLib->elements.push_back( readGeometry(child, state) );
        } else if (isName(child, "extra")) {
            geomLib->extras.push_back( readExtra(child,state) );
        }
    }
    return geomLib;
}

Dae::VisualScene* ColladaLoader::readVisualScene(xercesc::DOMElement* element, ParserState& state){
    Dae::VisualScene *scene = state.make<Dae::VisualScene>();
    scene->id = getAttrib(element, "id");
    scene->name = getAttrib(element, "name");
    state.add(scene, scene->id, "");
    ScopedPush p(scene, state);

    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "asset")) {
            scene->asset = readAsset(child, state);
        } else if (isName(child, "node")) {
            scene->nodes.push_back( readNode(child, state) );
        } else if (isName(child, "evaluate_scene")) {
            scene->evaluateScenes.push_back( readEvaluateScene(child, state) );
        } else if (isName(child, "extra")) {
            scene->extras.push_back( readExtra(child,state) );
        }
    }
    return scene;
}

Dae::Library<Dae::VisualScene>* ColladaLoader::readLibraryVisualScenes(DOMElement* element, ColladaLoader::ParserState& state) {
    Dae::Library<Dae::VisualScene> *visualSceneLib = state.make< Dae::Library<Dae::VisualScene> >();
    visualSceneLib->id = getAttrib(element, "id");
    visualSceneLib->name = getAttrib(element, "name");
    state.add(visualSceneLib, visualSceneLib->id, "");
    ScopedPush p(visualSceneLib, state);

    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "asset")) {
            visualSceneLib->asset = readAsset(child, state);
        } else if (isName(child, "visual_scene")) {
            visualSceneLib->elements.push_back( readVisualScene(child, state) );
        } else if (isName(child, "extra")) {
            visualSceneLib->extras.push_back( readExtra(child,state) );
        }
    }
    return visualSceneLib;
}

Dae::Library<Dae::Effect>* ColladaLoader::readLibraryEffects(DOMElement* element, ColladaLoader::ParserState& state) {
    Dae::Library<Dae::Effect> *matLib = state.make<Dae::Library<Dae::Effect> >();
    matLib->id = getAttrib(element, "id");
    matLib->name = getAttrib(element, "name");
    state.add(matLib, matLib->id, "");
    ScopedPush p(matLib, state);

    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "asset")) {
            matLib->asset = readAsset(child, state);
        } else if (isName(child, "effects")) {
            matLib->elements.push_back( readEffect(child, state) );
        } else if (isName(child, "extra")) {
            matLib->extras.push_back( readExtra(child,state) );
        }
    }
    return matLib;
}


Dae::Library<Dae::Material>* ColladaLoader::readLibraryMaterials(DOMElement* element, ColladaLoader::ParserState& state) {
    Dae::Library<Dae::Material> *matLib = state.make<Dae::Library<Dae::Material> >();
    matLib->id = getAttrib(element, "id");
    matLib->name = getAttrib(element, "name");
    state.add(matLib, matLib->id, "");
    ScopedPush p(matLib, state);

    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "asset")) {
            matLib->asset = readAsset(child, state);
        } else if (isName(child, "materials")) {
            matLib->elements.push_back( readMaterial(child, state) );
        } else if (isName(child, "extra")) {
            matLib->extras.push_back( readExtra(child,state) );
        }
    }
    return matLib;
}

Dae::InstanceVisualScene* ColladaLoader::readInstanceVisualScene(DOMElement* element, ColladaLoader::ParserState& state) {
    Dae::InstanceVisualScene *data = state.make<Dae::InstanceVisualScene>();
    data->sid = getAttrib(element, "sid");
    data->name = getAttrib(element, "name");
    data->url = getAttrib(element, "url");
    state.add(data,"", data->sid);
    ScopedPush p(data, state);

    // here we need to start instantiating stuff
    Dae::VisualScene *vscene = state.get<Dae::VisualScene>(data->url);

    Frame *world = state.wc->getWorldFrame();
    BOOST_FOREACH(Dae::Node* node, vscene->nodes){
        std::cout << "Node: " << node->name << std::endl;
        std::cout << "-- type: " << node->type << std::endl;


        // we need to traverse all nodes and add them to the workcell
        // nodes need to be mapped to frames, but in RobWork these frames would either be Fixed, Movable or Joints.
        // for now we use FixedFrame
        Transform3D<> transform;
        BOOST_FOREACH(Dae::Transform* t, node->transforms){transform = transform*t->transform;}
        FixedFrame *frame = new FixedFrame(node->name, transform);
        state.wc->getStateStructure()->addFrame(frame, world);


        // if a node has geometry then we define it as an Object
        if(node->igeometries.size()>0){
            std::vector<Model3D::Ptr> geoms;
            // now look for geometry and add it to the Object
            BOOST_FOREACH(Dae::InstanceGeometry* igeom, node->igeometries){
                std::cout << "igeom: " << igeom->name << " : " << igeom->url << std::endl;
                Dae::Geometry* geom = state.get<Dae::Geometry>(igeom->url);
                std::cout << "Geom: " << geom->name << std::endl;
                std::vector<Model3D::Ptr> rwgeom = makeRWGeometry(geom, state);
                BOOST_FOREACH(Model3D::Ptr g, rwgeom){
                    geoms.push_back( g );
                    if(getScene())
                        getScene()->addModel3D(geom->name, g, frame); // TODO: which layers should it be visible on
                }
            }

            if(geoms.size()>0){
                // create object
                std::cout << "Creating object" << std::endl;
                //Object::Ptr object = ownedPtr( new Object(frame, geoms) );
                //state.wc->add( object );
            }

        }
        // sensors

        // follow child nodes
        BOOST_FOREACH(Dae::InstanceNode* child, node->inodes){
            RW_WARN("Not impl yet!");
        }

    }

    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "extra")) {
            data->extras.push_back( readExtra(child,state) );
        }
    }

    return data;
}

void ColladaLoader::readScene(DOMElement* element, ColladaLoader::ParserState& state) {
    DOMElemChildVector domChildren(element);
    BOOST_FOREACH(DOMElement* child, domChildren){
        DEBUGL( XMLStr(child->getNodeName()).str() );
        if (isName(child, "instance_physics_scene")) {
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if (isName(child, "instance_visual_scene")) {
            readInstanceVisualScene(child, state);
        } else if (isName(child, "instance_kinematics_scene")) {
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if (isName(child, "extra")) {
            //matLib->extras.push_back( readExtra(child,state) );
            RW_WARN("NOT IMPLEMENTED YET!");
        }
    }
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
            // this is where the good shit happens
            readScene(child, state);
        } else if (isName(child, "extra")) {
            RW_WARN("NOT IMPLEMENTED YET!");
        } else if (isName(child, "library_materials")) {
            state.data.libMaterials.push_back( readLibraryMaterials(child, state) );
        } else if (isName(child, "library_effects")) {
            state.data.libEffects.push_back( readLibraryEffects(child, state) );
        }
    }
}

rw::models::WorkCell::Ptr ColladaLoader::getWorkCell(){
    return _pstate.wc;
}

void ColladaLoader::readColladaWorkCell(DOMElement* element) {
    DEBUGL("readColladaWorkCell");
    ParserState &collada = _pstate;
    collada.wc = ownedPtr(new WorkCell("nwc"));
    if(getScene()!=NULL)
        getScene()->setWorkCell( collada.wc );
    // potentially make a WorkCellScene such as to avoid
    //std::vector<PluginFactory<T>::Ptr> factories = SceneGraphRobWork::getInstance()->getPluginRepository().getPlugins<rw::graphics::SceneGraph>();

    if (isName(element, "COLLADA") ) {
        readCollada(element, collada);
    }

    _workcell = collada.wc;
}

