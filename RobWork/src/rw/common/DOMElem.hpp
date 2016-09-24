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

#ifndef RW_COMMON_DOMELEM_HPP
#define RW_COMMON_DOMELEM_HPP

#include <rw/common/Ptr.hpp>
#include <string>
#include <vector>

namespace rw {
namespace common {
	/**
	 * @brief an wrapper interface for easy access to XML DOM parser. This require an
	 * active back-end that does the actual parsing and validation.
	 *
	 * The DOMElem consist of a \<name\>, \<value\> and a DOMElem list of \<attributes\> and \<children\>. The value is a
	 * string and can (for convenience) be extracted as
	 * different primitive values on the interface eg. double or std::vector\<double\>.
	 * Attributes are DOMElem that does not have any children. Children are DOMElem that
	 * might contain other children.
	 *
	 */
	class DOMElem {
	protected:
		DOMElem(){};


	public:
		//! smart pointer type
		typedef rw::common::Ptr<DOMElem> Ptr;

		// forward declaration
		class Iterator;

		//! DOMElem Iterator Pair type
		typedef std::pair<class Iterator, class Iterator> IteratorPair;

		//struct IteratorPair;

		//! destructor
		virtual ~DOMElem(){};

		// get data and check name
		/**
		 * @brief test if name of this elem equals \b elemname
		 * @param elemname [in] string name to test with
		 * @return true if elemname equals name of this DOMElem, false otherwise
		 */
		virtual bool isName(const std::string& elemname) const = 0;

		/**
		 * @brief get name of this DOMElem
		 * @return name of DOMElem
		 */
		virtual const std::string& getName() const = 0;

		/**
		 * @brief get string value of this DOMElem
		 * @return string value of this elem
		 */
		virtual std::string getValue() const = 0;

		/**
		 * @brief get value as an integer, throws an exception if this is not an
		 * int value.
		 * @return the int value
		 */
		virtual int getValueAsInt() const = 0;

		/**
		 * @brief get value as an double floating point, throws an exception if this is not an
		 * double value.
		 * @return the double value
		 */
		virtual double getValueAsDouble() const = 0;

		/**
		 * @brief get value as a list of strings, throws an exception if this is not a
		 * list of strings. The default string seperator is semicolon (;).
		 * @return list of strings
		 */
		virtual std::vector<std::string> getValueAsStringList(char stringseperator = ';') const = 0;

		/**
		 * @brief get value as a list of doubles, throws an exception if this is not a
		 * list of doubles. The default string seperator is space ( ).
		 * @return list of doubles
		 */
		virtual std::vector<double> getValueAsDoubleList() const = 0;

		/**
		 * @brief get value as a list of \b N doubles, throws an exception if this is not a
		 * list of \b N doubles. The default string seperator is space ( ).
		 * @return list of N doubles
		 */
		virtual std::vector<double> getValueAsDoubleList(int N) const = 0;

		// get data as specific type. The funtion will try and parse the string into T
		//template<class T>
		//T getValueAs();

		// get elements that searches a specific key
		/**
		 * @brief get a child with a specific name. If more children with the same name occur then
		 * it cannot be guaranteed which is returned. If optional is false then an exception will be thrown
		 * if the child cannot be found. If optional is true then NULL is returned if child is not found.
		 * @param name [in] name of the child to find
		 * @param optional [in]
		 * @return the DOMElem child if found, else either NULL (is optional) or an exception is thrown.
		 */
		virtual rw::common::Ptr<DOMElem> getChild(const std::string& name, bool optional=false) = 0;

		/**
		 * @brief get a attribute with a specific name. If more attributes with the same name occur then
		 * it cannot be guaranteed which is returned. If optional is false then an exception will be thrown
		 * if the child cannot be found. If optional is true then NULL is returned if child is not found.
		 * @param name [in] name of the attribute to find
		 * @param optional [in]
		 * @return the DOMElem attribute if found, else either NULL (is optional) or an exception is thrown.
		 */
		virtual rw::common::Ptr<DOMElem> getAttribute(const std::string& name, bool optional=false) = 0;

		/**
		 * @brief test if this DOMElem has a child by name \b name.
		 * @param name [in] name of the child DOMElem
		 * @return true if this DOMElem has a child with name \b name
		 */
		virtual bool hasChild(const std::string& name) const = 0;

		/**
		 * @brief test if this DOMElem has any children
		 * @return true if this DOMElem has any children
		 */
		virtual bool hasChildren() const = 0;

		/**
		 * @brief test if this DOMElem has an attribute by name \b name.
		 * @param name [in] name of the attribute DOMElem
		 * @return true if this DOMElem has an attribute with name \b name
		 */
		virtual bool hasAttribute(const std::string& name) const = 0;

		/**
		 * @brief Get the value of attribute as a string.
		 * @param name [in] the name of the attribute.
		 * @return the value.
		 */
		std::string getAttributeValue(const std::string& name){ return getAttribute(name)->getValue(); }
		/**
		 * \copydoc getAttributeValue(const std::string&)
		 * @param default_value [in] a default value to return if attibute is not found.
		 */
		std::string getAttributeValue(const std::string& name, const std::string& default_value){
			if( DOMElem::Ptr attrib = getAttribute(name,true) )
				return attrib->getValue();
			return default_value;
		}

		/**
		 * @brief Get the value of attribute as an integer.
		 * @param name [in] the name of the attribute.
		 * @return the value.
		 */
		int getAttributeValueAsInt(const std::string& name){ return getAttribute(name)->getValueAsInt(); }
		/**
		 * \copydoc getAttributeValueAsInt(const std::string&)
		 * @param default_value [in] a default value to return if attibute is not found.
		 */
		int getAttributeValueAsInt(const std::string& name, int default_value){
			if( DOMElem::Ptr attrib = getAttribute(name,true) )
				return attrib->getValueAsInt();
			return default_value;
		}

		/**
		 * @brief Get the value of attribute as an double.
		 * @param name [in] the name of the attribute.
		 * @return the value.
		 */
		double getAttributeValueAsDouble(const std::string& name){ return getAttribute(name)->getValueAsDouble(); }
		/**
		 * \copydoc getAttributeValueAsDouble(const std::string&)
		 * @param default_value [in] a default value to return if attibute is not found.
		 */
		double getAttributeValueAsDouble(const std::string& name, double default_value){
			if( DOMElem::Ptr attrib = getAttribute(name,true) )
				return attrib->getValueAsDouble();
			return default_value;
		}

		/**
		 * @brief Get the value as a boolean.
		 * @return boolean value.
		 */
		bool getValueAsBool();

		/**
		 * @brief Get the value of attribute as an boolean.
		 * @param name [in] the name of the attribute.
		 * @return the value.
		 */
		bool getAttributeValueAsBool(const std::string& name){
			return getAttribute(name)->getValueAsBool();
		}
		/**
		 * \copydoc getAttributeValueAsBool(const std::string&)
		 * @param default_value [in] a default value to return if attibute is not found.
		 */
		bool getAttributeValueAsBool(const std::string& name, bool default_value){
			if( DOMElem::Ptr attrib = getAttribute(name,true) )
				return attrib->getValueAsBool();
			return default_value;
		}

		/**
		 * @brief Get iterator to first child element.
		 * @return iterator.
		 */
		virtual Iterator begin() = 0;

		/**
		 * @brief Get iterator to last child element.
		 * @return iterator.
		 */
		virtual Iterator end() = 0;

		/**
		 * @brief Get iterator to child elements.
		 * @return iterator pair for the first and last elements.
		 */
		virtual IteratorPair getChildren() = 0;

		/**
		 * @brief Get iterator to attributes.
		 * @return iterator pair for the first and last attributes.
		 */
		virtual IteratorPair getAttributes() = 0;


		// now comes the construction stuff.. so this should enable one to add elements and attributes
		/**
		 * @brief Add a child with an empty name.
		 * @return pointer to the new child element.
		 */
		virtual rw::common::Ptr<DOMElem> addChild(){ return addChild(""); }

		/**
		 * @brief Add a child element.
		 * @param name [in] name of child.
		 * @return pointer to the new child element.
		 */
		virtual rw::common::Ptr<DOMElem> addChild(const std::string& name) = 0;

		/**
		 * @brief Add an attribute element.
		 * @param name [in] name of the attribute.
		 * @return pointer to the new attribute.
		 */
		virtual rw::common::Ptr<DOMElem> addAttribute(const std::string& name) = 0;

		/**
		 * @brief Set the value of this element.
		 * @param val [in] new value.
		 */
		virtual void setValue(const std::string& val) = 0;

		/**
		 * @brief Set the name of this element.
		 * @param val [in] new name.
		 */
		virtual void setName(const std::string& val) = 0;

		//! @copydoc setValue(const std::string&)
		virtual void setValue(bool val);
		//! @copydoc setValue(const std::string&)
		virtual void setValue(int val);
		//! @copydoc setValue(const std::string&)
		virtual void setValue(double val);

	protected:

		/**
		 * @brief The DOMElem Iterator is initialized with a specific implementation
		 * of this interface.
		 */
		class ItImpl {
		public:

			//! destructor
			virtual ~ItImpl(){}

			//! clone this ItImpl object
			virtual ItImpl* clone() = 0;
			//! increment the iterator
			virtual void increment() = 0;
			//virtual void add(int right) = 0;
			//! get the element that the iterator is currently pointing to
			virtual DOMElem::Ptr getElem() = 0;
			//! test if two iterators are in same state
			virtual bool equal(ItImpl* ) const = 0;
		};

	public:

		/**
		 * @brief DOMElem iterator based on concrete ItImpl implementations.
		 */
		class Iterator  {
			ItImpl *_impl;
		public:
			/** Iterator category. */
			typedef std::forward_iterator_tag iterator_category;

			/** Value type. */
			typedef rw::common::Ptr<DOMElem> value_type;

			/** Pointer type. */
			typedef rw::common::Ptr<DOMElem> pointer;

			/** Reference type. */
			typedef rw::common::Ptr<DOMElem> reference;

			/** Difference type. */
			typedef ptrdiff_t difference_type;

			//! constructor
			Iterator():_impl(NULL){}
			//! constructor
			Iterator(ItImpl *impl):_impl(impl){}
			//! constructor
			Iterator(Iterator const& right) : _impl(right._impl->clone()) {}
			//! destructor
			virtual ~Iterator() { delete _impl; }

			/**
			 * @brief Assignment operator.
			 * @param right [in] the Iterator to assign.
			 * @return a reference to this iterator (for chaining).
			 */
			Iterator& operator=(Iterator const& right)
			{
				delete _impl;
				_impl = right._impl->clone();
				return *this;
			}


			//! @brief Reference to the T element
			virtual rw::common::Ptr<DOMElem> operator*() {
				if(_impl)
					return _impl->getElem();
				return NULL;
			}

			//! @brief Pointer to the T element
			virtual rw::common::Ptr<DOMElem> operator->() {
				if(_impl)
					return _impl->getElem();
				return NULL;
			}

			/**
			 * @brief Increments the position of the iterator
			 * @return Reference to the incremented iterator
			 */
			Iterator& operator++(){
				_impl->increment();
				return *this;
			}

			/**
			 * @brief Increments the position of the iterator
			 * @return the ConcatVectorIterator with the value before the incrementation
			 */
			/*
			Iterator& operator++(int right){
				_impl->add(right);
				 return *this;
			}
	*/
			/**
			 * @brief Tests whether the positions of two iterators are equal
			 * @param other [in] ConcatVectorIterator to compare with
			 * @return true if equal
			 */
			virtual bool operator==(const Iterator& other) const{ return _impl->equal(other._impl); }

			/**
			 * @brief Tests whether the positions of two iterators are unequal
			 * @param other [in] ConcatVectorIterator to compare with
			 * @return true if unequal
			 */
			virtual bool operator!=(const Iterator& other) const{ return !_impl->equal(other._impl); }
		};



	};
}
}
#endif
