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

#include <cstdlib>
#include <cmath>
#include <string>

#include <boost/any.hpp>
#include <cstdio>
#include <fstream>
#include <rw/common/macros.hpp>
#include <boost/any.hpp>
#include "Archive.hpp"

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
		virtual ~Serializable(){};
	//protected:
		//friend Archive::Access;
		virtual void read(class InputArchive& iarchive, const std::string& id) = 0;
		virtual void write(class OutputArchive& iarchive, const std::string& id) const = 0;
	};

	/**
	 * @brief provide generic handler interface for serialization purposes. To enable serialization
	 * of some class A one could either inherit from Serializable or provide overloaded methods to
	 * \code
	 * void read(A& object, InputArchive& iarchive, const std::string& id){ .. your serialize code ... }
	 * void read(A& object, InputArchive& iarchive, const std::string& id){ .. your serialize code ... }
	 *
	 * @param data
	 * @param iarchive
	 * @param id
	 */
	namespace serialization {

		template<class T>
		void read(T& data, class InputArchive& iarchive, const std::string& id){
			RW_THROW("No overloaded method to deserialize class: " << typeid(T).name());
		}

		template<class T>
		void write(const T& data, class OutputArchive& iarchive, const std::string& id){
			RW_THROW("No overloaded method to serialize class: " << typeid(T).name());
		}

	}

}}
#endif
