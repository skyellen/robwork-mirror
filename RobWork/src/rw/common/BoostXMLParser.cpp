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

#include "BoostXMLParser.hpp"


#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include <rw/common/IOUtil.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw;
using namespace rw::common;

//////////////// stuff for loader

BoostXMLParser::BoostInitializer::BoostInitializer() {
	static bool done = false;
	if (!done) {
		done = true;
#if (BOOST_VERSION<105600)
                boost::property_tree::xml_parser::xmlattr<char>();
                boost::property_tree::xml_parser::xmltext<char>();
                boost::property_tree::xml_parser::xmlcomment<char>();
                boost::property_tree::xml_parser::xmldecl<char>();
#else
                boost::property_tree::xml_parser::xmlattr<std::string>();
                boost::property_tree::xml_parser::xmltext<std::string>();
                boost::property_tree::xml_parser::xmlcomment<std::string>();
                boost::property_tree::xml_parser::xmldecl<std::string>();
#endif
        }
}

const BoostXMLParser::BoostInitializer BoostXMLParser::initializer;

BoostXMLParser::BoostXMLParser():_debug(false){
	_tree = ownedPtr( new boost::property_tree::ptree() );
	_root = rw::common::ownedPtr(new BoostDOMElem("",_tree, NULL, _tree, this));
}

void BoostXMLParser::load(const std::string& filename){
	std::string file = IOUtil::getAbsoluteFileName(filename);
	_tree = ownedPtr( new boost::property_tree::ptree() );
    try {
        read_xml(file, *_tree, boost::property_tree::xml_parser::trim_whitespace);
        // create root element
        _root = rw::common::ownedPtr(new BoostDOMElem("",_tree, NULL, _tree, this));
    } catch (const boost::property_tree::xml_parser_error& e) {
        RW_THROW(e.message() << ". \n\tIn line: " << e.line() << " \n\t In file: " << e.filename());

    } catch (const boost::property_tree::ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}

void BoostXMLParser::load(std::istream& input){
	_tree = ownedPtr( new boost::property_tree::ptree() );
	try {
        read_xml(input, *_tree, boost::property_tree::xml_parser::trim_whitespace);
        // create root element
        _root = rw::common::ownedPtr(new BoostDOMElem("",_tree, NULL, _tree, this));
    } catch (const boost::property_tree::xml_parser_error& e) {
        RW_THROW(e.message() << ". \n\tIn line: " << e.line() << " \n\t In file: " << e.filename());
    } catch (const boost::property_tree::ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}

void BoostXMLParser::save(const std::string& filename){
    try {
#if (BOOST_VERSION<105600)
    	boost::property_tree::xml_writer_settings<char> settings(' ', 1);
#else
        boost::property_tree::xml_writer_settings<std::string> settings(' ', 1);
#endif
       write_xml(filename, *_tree, std::locale(), settings);
    } catch (const boost::property_tree::ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}

void BoostXMLParser::save(std::ostream& output){
    try {
#if (BOOST_VERSION<105600)
    	boost::property_tree::xml_writer_settings<char> settings(' ', 1);
#else
    	boost::property_tree::xml_writer_settings<std::string> settings(' ', 1);
#endif
        write_xml(output, *_tree, settings);
    } catch (const boost::property_tree::ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}


std::vector<std::string> BoostDOMElem::getValueAsStringList(char stringseperator) const {
	const std::string value = _node->get_value<std::string>();
    std::vector<std::string> values;
    boost::char_separator<char> sep(std::string(1,stringseperator).c_str());
    boost::tokenizer< boost::char_separator<char> > tok(value, sep);
    values.assign(tok.begin(),tok.end());
    return values;
}

std::vector<double> BoostDOMElem::getValueAsDoubleList() const {
    std::istringstream buf(_node->get_value<std::string>());
    std::vector<double> values;
    std::string str;
    while( buf >> str ){
    	double val = boost::lexical_cast<double>(str);
        values.push_back(val);
    }
    return values;
}

std::vector<double> BoostDOMElem::getValueAsDoubleList(int size) const {
    std::istringstream buf(_node->get_value<std::string>());
    std::vector<double> values;
    std::string str;
    while( buf >> str ){
    	double val = boost::lexical_cast<double>(str);
        values.push_back(val);
    }
    if(values.size()!= (size_t)size)
    	RW_THROW("Error parsing element \""<< _name <<"\" expected "<<size<< " doubles, got " << values.size());
    return values;
}

std::string BoostDOMElem::getValue() const {
    std::string val = _node->get_value<std::string>();
    if(_parser->isDebug())
        Log::debugLog() << val;

    return _node->get_value<std::string>();
}

int BoostDOMElem::getValueAsInt() const {
    int val = _node->get_value<int>();
    if(_parser->isDebug())
        Log::debugLog() << val;
    return val;
}

double BoostDOMElem::getValueAsDouble() const {
    double val = _node->get_value<double>();
    if(_parser->isDebug())
        Log::debugLog() << val;
    return val;
}

bool BoostDOMElem::hasChild(const std::string& name) const {
	boost::optional<boost::property_tree::ptree&> child = _node->get_child_optional(name);
	if(child)
		return true;
	return false;
}

bool BoostDOMElem::hasAttribute(const std::string& name) const {
	boost::optional<boost::property_tree::ptree&> attr = _node->get_child_optional("<xmlattr>");
	if(attr){
		boost::optional<boost::property_tree::ptree&> child = _node->get_child("<xmlattr>").get_child_optional(name);
		if(child)
			return true;
	}
	return false;
}


DOMElem::Ptr BoostDOMElem::getChild(const std::string& name, bool optional){
    if(_parser->isDebug())
        Log::debugLog() << "Child: " << name << "\n";

	boost::optional<boost::property_tree::ptree&> child = _node->get_child_optional(name);
	if(child)
		return rw::common::ownedPtr(new BoostDOMElem(name, &child.get(), _node, this->getRoot(), _parser) );

	if(!optional){
		RW_THROW("Parse error: Child \"" << name << "\" of element " << getName() << " does not exist!");
	}
	return NULL;
}

DOMElem::Ptr BoostDOMElem::getAttribute(const std::string& name, bool optional){
    if(_parser->isDebug())
        Log::debugLog() << "Attr: " << name << "\n";

	boost::optional<boost::property_tree::ptree&> attr = _node->get_child_optional("<xmlattr>");
	if(attr){
		boost::optional<boost::property_tree::ptree&> child = _node->get_child("<xmlattr>").get_child_optional(name);
		if(child)
			return rw::common::ownedPtr(new BoostDOMElem(name, &child.get(), _node, this->getRoot(), _parser) );
	}
	if(!optional){
		RW_THROW("Parse error: Attribute \"" << name << "\" of element " << getName() << " does not exist!");
	}
	return NULL;
	//boost::property_tree::ptree *child = &(_node->get_child("<xmlattr>").get_child(name));
	//return rw::common::ownedPtr(new BoostDOMElem(name, child) );
}

DOMElem::IteratorPair BoostDOMElem::getChildren(){
	return std::make_pair(
			DOMElem::Iterator(new ElemIterImpl(_node->begin(),_node->end(), _node, _root,_parser)),
			DOMElem::Iterator(new ElemIterImpl(_node->end()  ,_node->end(), _node, _root,_parser)) );
}

DOMElem::IteratorPair BoostDOMElem::getAttributes(){
	boost::property_tree::ptree& attribs = _node->get_child("<xmlattr>");
	return std::make_pair(
			DOMElem::Iterator(new ElemIterImpl(attribs.begin(),attribs.end(), _node, _root,_parser)),
			DOMElem::Iterator(new ElemIterImpl(attribs.end()  ,attribs.end(), _node, _root,_parser)) );
}

rw::common::Ptr<DOMElem> BoostDOMElem::addChild(const std::string& name){
	boost::property_tree::ptree &child = _node->add_child(name, boost::property_tree::ptree() );
	return rw::common::ownedPtr(new BoostDOMElem(name, &child, _node, this->getRoot(),_parser) );
	//return this->getChild(name);
}

rw::common::Ptr<DOMElem> BoostDOMElem::addAttribute(const std::string& name){
	std::string namePath = std::string("<xmlattr>.") + name;
	_node->add_child(namePath, boost::property_tree::ptree() );
	return this->getAttribute(name);
}

void BoostDOMElem::setValue(const std::string& val){
	_node->data() = val;
}

void BoostDOMElem::setName(const std::string& name){
	// TODO: we cannot set name directly in boost property tree, so instead
	// we insert new node, swap with old and erase old node.

	// first be sure we are not at the root node
	if(_parent==NULL){
		RW_THROW("you cannot set name of root");
	}

    // create a new node
	    // swap the current node with the new
    boost::property_tree::ptree &newchild = _parent->add_child(name, boost::property_tree::ptree() );
    _node->swap( newchild );
    _name = name;

    boost::property_tree::ptree::iterator child_iter = _parent->begin();
    while(child_iter!=_parent->end()){
        if( child_iter->second == *_node ){
            // we found the child, erase it and quit
            _node = rw::common::Ptr< boost::property_tree::ptree >( &newchild );
            _parent->erase(child_iter);
            return;
        }
        ++child_iter;
    }
    RW_THROW("Could not find child in parent!!!!");
}

DOMElem::Iterator BoostDOMElem::begin(){
	return DOMElem::Iterator(new ElemIterImpl(_node->begin(),_node->end(), _parent, _root,_parser) );
}
DOMElem::Iterator BoostDOMElem::end(){
	return DOMElem::Iterator(new ElemIterImpl(_node->end(),_node->end(), _parent, _root,_parser) );
}


bool BoostDOMElem::hasChildren() const{
	return !_node->empty();
}
