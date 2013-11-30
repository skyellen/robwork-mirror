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

#ifndef RW_COMMON_BOOSTXMLPARSER_HPP
#define RW_COMMON_BOOSTXMLPARSER_HPP

#include <string>
#include <vector>
#include <list>

#include <assert.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "DOMParser.hpp"

namespace rw {
namespace common {

	/**
	 * @brief a DOMParser implementation based on the Boost xml parser in propertytree
	 */
	class BoostXMLParser: public DOMParser {
	public:
		//! constructor
		BoostXMLParser();
		// loading and saving
		void load(const std::string& filename);
		void load(std::istream& input);
		void save(const std::string& filename);
		void save(std::ostream& input);

		DOMElem::Ptr getRootElement(){ return _root;};

		// extra stuff that can be added in top of document

	private:
		DOMElem::Ptr _root;
		rw::common::Ptr< boost::property_tree::ptree > _tree;
	};

	class BoostDOMElem : public DOMElem {
	public:
		BoostDOMElem(const std::string& key,
		             rw::common::Ptr<boost::property_tree::ptree> ptree,
		             rw::common::Ptr< boost::property_tree::ptree > parent,
		             rw::common::Ptr< boost::property_tree::ptree > root):
			_name(key),
			_node(ptree),
			_parent(parent),
			_root(root)
		{};

	public:

		//! @copydoc DOMElem::isName
		bool isName(const std::string& name) const { return _name == name; }
		//! @copydoc DOMElem::setName
		void setName(const std::string& name);

		//! @copydoc DOMElem::getName
		const std::string& getName() const { return _name; }
		//! @copydoc DOMElem::getValue
		std::string getValue() const { return _node->get_value<std::string>(); }

		//! @copydoc DOMElem::getValueAsInt
		int getValueAsInt() const { return _node->get_value<int>(); }
		//! @copydoc DOMElem::getValueAsDouble
		double getValueAsDouble() const { return _node->get_value<double>(); }
		//! @copydoc DOMElem::getValueAsStringList
		std::vector<std::string> getValueAsStringList(char stringseperator = ';') const ;
		std::vector<double> getValueAsDoubleList() const ;
		std::vector<double> getValueAsDoubleList(int size) const ;

		// get elements that searches a specific key
		DOMElem::Ptr getChild(const std::string& name, bool optional=false);
		DOMElem::Ptr getAttribute(const std::string& name, bool optional=false);


		bool hasChildren() const;
		bool hasChild(const std::string& name) const ;
		bool hasAttribute(const std::string& name) const ;

		rw::common::Ptr<DOMElem> addChild(const std::string& name);
		rw::common::Ptr<DOMElem> addAttribute(const std::string& name);
		void setValue(const std::string& val);


		// get child elements

		DOMElem::IteratorPair getChildren();

		DOMElem::IteratorPair getAttributes();

		DOMElem::Iterator begin();
		DOMElem::Iterator end();

		rw::common::Ptr< boost::property_tree::ptree > getRoot(){ return _root; } ;

	public:

		class ElemIterImpl : public DOMElem::ItImpl {
		public:
			boost::property_tree::ptree::iterator _begin,_end;
			rw::common::Ptr< boost::property_tree::ptree > _parent,_root;
			ElemIterImpl(boost::property_tree::ptree::iterator begin,
			               boost::property_tree::ptree::iterator end,
			               rw::common::Ptr< boost::property_tree::ptree > parent,
			               rw::common::Ptr< boost::property_tree::ptree > root):
				_begin(begin),_end(end),_parent(parent),_root(root)
			{
				while(_begin!=_end && _begin->first=="<xmlattr>")
					_begin++;
			}

			ItImpl* clone(){
				return new ElemIterImpl(_begin, _end, _parent, _root);
			}

			void increment(){
				_begin++;
				while(_begin!=_end && _begin->first=="<xmlattr>")
					_begin++;
			}

			//void add(int right){ _begin+=right; }

			DOMElem::Ptr getElem(){
				return rw::common::ownedPtr( new BoostDOMElem( _begin->first, &(_begin->second), _parent, _root) );
			}

			bool equal(ItImpl* iter) const{
				return _begin == ((ElemIterImpl*)iter)->_begin;
			}
		};

	private:
		std::string _name;
		rw::common::Ptr<boost::property_tree::ptree> _node;
		rw::common::Ptr<boost::property_tree::ptree> _parent;
		rw::common::Ptr< boost::property_tree::ptree > _root;
	};

}} //namespace
#endif
