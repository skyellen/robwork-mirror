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

		void setDebug(bool debug){_debug=debug;};

		bool isDebug(){return _debug;};
		DOMElem::Ptr getRootElement(){ return _root;};

		/**
		 * @brief Utility class which initializes Boost local static variables.
		 *
		 * If the BoostXMLParser is used outside main (as a part of global initialization/destruction), the BoostInitializer
		 * should be used explicitly to control the static initialization/destruction order.
		 *
		 * Notice that the BoostInitializer is automatically defined as a global variable, hence it should not
		 * be necessary to specify the initializer explicitly if BoostXMLParser is to be used in local static
		 * initialization/destruction.
		 */
		class BoostInitializer {
		public:
		    //! @brief Initializes BoostXMLParser when constructed.
			BoostInitializer();
		};

	private:
		static const BoostInitializer initializer;
		bool _debug;
		DOMElem::Ptr _root;
		rw::common::Ptr< boost::property_tree::ptree > _tree;
	};

	class BoostDOMElem : public DOMElem {
	public:
		BoostDOMElem(const std::string& key,
		             rw::common::Ptr<boost::property_tree::ptree> ptree,
		             rw::common::Ptr< boost::property_tree::ptree > parent,
		             rw::common::Ptr< boost::property_tree::ptree > root,
		             BoostXMLParser* parser):
			_name(key),
			_node(ptree),
			_parent(parent),
			_root(root),
			_parser(parser)
		{};

	public:

		//! @copydoc DOMElem::isName
		bool isName(const std::string& name) const { return _name == name; }
		//! @copydoc DOMElem::setName
		void setName(const std::string& name);

		//! @copydoc DOMElem::getName
		const std::string& getName() const { return _name; }
		//! @copydoc DOMElem::getValue
		std::string getValue() const;

		//! @copydoc DOMElem::getValueAsInt
		int getValueAsInt() const ;
		//! @copydoc DOMElem::getValueAsDouble
		double getValueAsDouble() const ;
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
			BoostXMLParser *_parser;

			ElemIterImpl(boost::property_tree::ptree::iterator begin,
			               boost::property_tree::ptree::iterator end,
			               rw::common::Ptr< boost::property_tree::ptree > parent,
			               rw::common::Ptr< boost::property_tree::ptree > root,
			               BoostXMLParser *parser):
				_begin(begin),_end(end),_parent(parent),_root(root),_parser(parser)
			{
				while(_begin!=_end && _begin->first=="<xmlattr>")
					_begin++;
			}

			ItImpl* clone(){
				return new ElemIterImpl(_begin, _end, _parent, _root,_parser);
			}

			void increment(){
				_begin++;
				while(_begin!=_end && _begin->first=="<xmlattr>")
					_begin++;
			}

			//void add(int right){ _begin+=right; }

			DOMElem::Ptr getElem(){
				return rw::common::ownedPtr( new BoostDOMElem( _begin->first, &(_begin->second), _parent, _root, _parser) );
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
		BoostXMLParser *_parser;
	};

}} //namespace
#endif
