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

#include <rw/common/IOUtil.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw;
using namespace rw::common;

//////////////// stuff for loader

BoostXMLParser::BoostXMLParser(){
	_tree = ownedPtr( new boost::property_tree::ptree() );
	_root = rw::common::ownedPtr(new BoostDOMElem("",_tree, _tree));
}

void BoostXMLParser::load(const std::string& filename){
	std::string file = IOUtil::getAbsoluteFileName(filename);
	_tree = ownedPtr( new boost::property_tree::ptree() );
    try {
        read_xml(file, *_tree, boost::property_tree::xml_parser::trim_whitespace);
        // create root element
        _root = rw::common::ownedPtr(new BoostDOMElem("",_tree, _tree));
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
        _root = rw::common::ownedPtr(new BoostDOMElem("",_tree, _tree));
    } catch (const boost::property_tree::ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}

void BoostXMLParser::save(const std::string& filename){
    try {
    	boost::property_tree::xml_writer_settings<char> settings(' ', 1);
    	write_xml(filename, *_tree, std::locale(), settings);
    } catch (const boost::property_tree::ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}

void BoostXMLParser::save(std::ostream& output){
    try {
    	boost::property_tree::xml_writer_settings<char> settings(' ', 1);
        write_xml(output, *_tree, settings);
    } catch (const boost::property_tree::ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}




std::vector<std::string> BoostDOMElem::getValueAsStringList(char stringseperator) const {
	return std::vector<std::string>();
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
	boost::optional<boost::property_tree::ptree&> child = _node->get_child_optional(name);
	if(child)
		return rw::common::ownedPtr(new BoostDOMElem(name, &child.get(), this->getRoot()) );

	if(!optional){
		RW_THROW("Parse error: Child \"" << name << "\" of element " << getName() << " does not exist!");
	}
	return NULL;
}

DOMElem::Ptr BoostDOMElem::getAttribute(const std::string& name, bool optional){
	boost::optional<boost::property_tree::ptree&> attr = _node->get_child_optional("<xmlattr>");
	if(attr){
		boost::optional<boost::property_tree::ptree&> child = _node->get_child("<xmlattr>").get_child_optional(name);
		if(child)
			return rw::common::ownedPtr(new BoostDOMElem(name, &child.get(), this->getRoot()) );
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
			DOMElem::Iterator(new ElemIterImpl(_node->begin(),_node->end(), _root)),
			DOMElem::Iterator(new ElemIterImpl(_node->end()  ,_node->end(), _root)) );
}

DOMElem::IteratorPair BoostDOMElem::getAttributes(){
	boost::property_tree::ptree& attribs = _node->get_child("<xmlattr>");
	return std::make_pair(
			DOMElem::Iterator(new ElemIterImpl(attribs.begin(),attribs.end(), _root)),
			DOMElem::Iterator(new ElemIterImpl(attribs.end()  ,attribs.end(), _root)) );
}

rw::common::Ptr<DOMElem> BoostDOMElem::addChild(const std::string& name){
	boost::property_tree::ptree &child = _node->add_child(name, boost::property_tree::ptree() );
	return rw::common::ownedPtr(new BoostDOMElem(name, &child, this->getRoot()) );
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



DOMElem::Iterator BoostDOMElem::begin(){
	return DOMElem::Iterator(new ElemIterImpl(_node->begin(),_node->end(), _root) );
}
DOMElem::Iterator BoostDOMElem::end(){
	return DOMElem::Iterator(new ElemIterImpl(_node->end(),_node->end(), _root) );
}


bool BoostDOMElem::hasChildren() const{
	return !_node->empty();
}

