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

#include <boost/property_tree/ptree_fwd.hpp>

#include "DOMParser.hpp"
#include "DOMElem.hpp"

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
		//! @copydoc DOMParser::load(const std::string&)
		void load(const std::string& filename);
		//! @copydoc DOMParser::load(std::istream&)
		void load(std::istream& input);
		//! @copydoc DOMParser::save(const std::string&)
		void save(const std::string& filename);
		//! @copydoc DOMParser::save(std::ostream&)
		void save(std::ostream& input);

		//! @copydoc DOMParser::setDebug
		void setDebug(bool debug){_debug=debug;}

		/**
		 * @brief Get status of debugging.
		 * @return true if enabled, false if disabled.
		 */
		bool isDebug(){return _debug;}

		//! @copydoc DOMParser::getRootElement
		DOMElem::Ptr getRootElement(){ return _root;}

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

	/**
	 * @brief DOMElem based on Boost PropertyTree xml parser.
	 */
	class BoostDOMElem : public DOMElem {
	public:
		/**
		 * @brief Construct new element.
		 * @param key [in] the key.
		 * @param ptree [in] boost property tree.
		 * @param parent [in] boost property tree.
		 * @param root [in] boost property tree.
		 * @param parser [in] the BoostXMLParser.
		 */
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
		{}

	public:

		//! @copydoc DOMElem::isName
		bool isName(const std::string& elemname) const { return _name == name; }
		//! @copydoc DOMElem::setName
		void setName(const std::string& val);

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
		//! @copydoc DOMElem::getValueAsDoubleList()
		std::vector<double> getValueAsDoubleList() const ;
		//! @copydoc DOMElem::getValueAsDoubleList(int)
		std::vector<double> getValueAsDoubleList(int size) const ;

		// get elements that searches a specific key
		//! @copydoc DOMElem::getChild
		DOMElem::Ptr getChild(const std::string& name, bool optional=false);
		//! @copydoc DOMElem::getAttribute
		DOMElem::Ptr getAttribute(const std::string& name, bool optional=false);


		//! @copydoc DOMElem::hasChildren
		bool hasChildren() const;
		//! @copydoc DOMElem::hasChild
		bool hasChild(const std::string& name) const ;
		//! @copydoc DOMElem::hasAttribute
		bool hasAttribute(const std::string& name) const ;

		//! @copydoc DOMElem::addChild(const std::string&)
		rw::common::Ptr<DOMElem> addChild(const std::string& name);

		//! @copydoc DOMElem::addAttribute(const std::string&)
		rw::common::Ptr<DOMElem> addAttribute(const std::string& name);

		//! @copydoc DOMElem::setValue(const std::string&)
		void setValue(const std::string& val);


		// get child elements
		//! @copydoc DOMElem::getChildren
		DOMElem::IteratorPair getChildren();

		//! @copydoc DOMElem::getAttributes
		DOMElem::IteratorPair getAttributes();

		//! @copydoc DOMElem::begin
		DOMElem::Iterator begin();
		//! @copydoc DOMElem::end
		DOMElem::Iterator end();

		/**
		 * @brief Get the root Boost PropertyTree element.
		 * @return pointer to the tree.
		 */
		rw::common::Ptr< boost::property_tree::ptree > getRoot(){ return _root; }

	private:
		class ElemIterImpl;

	private:
		std::string _name;
		rw::common::Ptr<boost::property_tree::ptree> _node;
		rw::common::Ptr<boost::property_tree::ptree> _parent;
		rw::common::Ptr< boost::property_tree::ptree > _root;
		BoostXMLParser *_parser;
	};

}} //namespace
#endif
