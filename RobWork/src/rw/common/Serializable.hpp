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

#ifndef RW_COMMON_SERIALIZABLE_HPP
#define RW_COMMON_SERIALIZABLE_HPP

#include <string>
#include <rw/common/macros.hpp>

namespace rw {
namespace common {

	class InputArchive;
	class OutputArchive;

	/**
	 * @brief interface for defining serialization of classes. If a class cannot inherit
	 * the Serializable because of non-access to code then one can instead provide
	 * overloaded read/write methods to perform the serialization.
	 */
	class Serializable {
	public:

		//! destructor
		virtual ~Serializable(){};

		/**
		 * Enable read-serialization of inherited class by implementing this method. Data is read
		 * from iarchive and filled into this object.
		 * @param iarchive [in] the InputArchive from which to read data.
		 * @param id [in] The id of the serialized sobject.
		 *
		 * @note the id can be empty in which case the overloaded method should provide
		 * a default identifier. E.g. the Vector3D class defined "Vector3D" as its default
		 * id.
		 */
		virtual void read(class InputArchive& iarchive, const std::string& id) = 0;

		/**
		 * Enable write-serialization of inherited class by implementing this method. Data is written
		 * to oarchive from this object.
		 * @param oarchive [out] the OutputArchive in which data should be written.
		 * @param id [in] The id of the serialized sobject.
		 *
		 * @note the id can be empty in which case the overloaded method should provide
		 * a default identifier. E.g. the Vector3D class defined "Vector3D" as its default
		 * id.
		 */
		virtual void write(class OutputArchive& oarchive, const std::string& id) const = 0;
	};

	/**
	 * @brief provide generic handler interface for serialization purposes. To enable serialization
	 * of some class MyClass one could either inherit from Serializable or provide overloaded methods to
	 * \code
	 * template<> void read(MyClass& object, InputArchive& iarchive, const std::string& id){ .. your serialize code ... }
	 * template<> void write(const MyClass& object, OutputArchive& oarchive, const std::string& id){ .. your serialize code ... }
	 * \endcode
	 */
	namespace serialization {

		/**
		 * Enable read-serialization of class T by overloading this method. Data is read
		 * from iarchive and filled into sobject.
		 * @param sobject [out] the object in which the data should be streamed into
		 * @param iarchive [in] the InputArchive from which to read data.
		 * @param id [in] The id of the serialized sobject.
		 *
		 * @note the id can be empty in which case the overloaded method should provide
		 * a default identifier. E.g. the Vector3D class defined "Vector3D" as its default
		 * id.
		 */
		template<class T>
		void read(T& sobject, class InputArchive& iarchive, const std::string& id){
			RW_THROW("No overloaded method to deserialize class: " << typeid(T).name());
		}

		/**
		 * Enable write-serialization of class T by overloading this method. Data is written
		 * to oarchive from the sobject.
		 * @param sobject [in] the object from which the data should be streamed.
		 * @param oarchive [out] the OutputArchive in which data should be written.
		 * @param id [in] The id of the serialized sobject.
		 *
		 * @note the id can be empty in which case the overloaded method should provide
		 * a default identifier. E.g. the Vector3D class defined "Vector3D" as its default
		 * id.
		 */
		template<class T>
		void write(const T& sobject, class OutputArchive& oarchive, const std::string& id){
			RW_THROW("No overloaded method to serialize class: " << typeid(T).name());
		}

	}

}}
#endif
