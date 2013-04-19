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

#ifndef RW_COMMON_OUTPUTARCHIVE_HPP
#define RW_COMMON_OUTPUTARCHIVE_HPP

#include "Archive.hpp"
#include "Serializable.hpp"
#include <boost/cstdint.hpp>
#include <boost/type_traits.hpp>

namespace rw {
namespace common {
	/**
	 * @brief serializable objects can be written to an output archive.
	 *
	 * This class define an interface for serializing data.
	 */
	class OutputArchive : public Archive {
	public:
		//! @brief destructor
		virtual ~OutputArchive(){};

		//! @copydoc Archive::open
		virtual void open(std::ostream& ofs) = 0;

		// utils to handle arrays
		virtual void writeEnterScope(const std::string& id) = 0;
		virtual void writeLeaveScope(const std::string& id) = 0;

		// writing primitives to archive
		virtual void write(bool val, const std::string& id) = 0;
		virtual void write(boost::int8_t val, const std::string& id) = 0;
		virtual void write(boost::uint8_t val, const std::string& id) = 0;
		virtual void write(boost::int16_t val, const std::string& id) = 0;
		virtual void write(boost::uint16_t val, const std::string& id) = 0;
		virtual void write(boost::int32_t val, const std::string& id) = 0;
		virtual void write(boost::uint32_t val, const std::string& id) = 0;
		virtual void write(boost::int64_t val, const std::string& id) = 0;
		virtual void write(boost::uint64_t val, const std::string& id) = 0;
		virtual void write(float val, const std::string& id) = 0;
		virtual void write(double val, const std::string& id) = 0;
		virtual void write(const std::string& val, const std::string& id) = 0;

		virtual void write(const std::vector<bool>& val, const std::string& id) = 0;
		virtual void write(const std::vector<boost::int8_t>& val, const std::string& id) = 0;
		virtual void write(const std::vector<boost::uint8_t>& val, const std::string& id) = 0;
		virtual void write(const std::vector<boost::int16_t>& val, const std::string& id) = 0;
		virtual void write(const std::vector<boost::uint16_t>& val, const std::string& id) = 0;
		virtual void write(const std::vector<boost::int32_t>& val, const std::string& id) = 0;
		virtual void write(const std::vector<boost::uint32_t>& val, const std::string& id) = 0;
		virtual void write(const std::vector<boost::int64_t>& val, const std::string& id) = 0;
		virtual void write(const std::vector<boost::uint64_t>& val, const std::string& id) = 0;
		virtual void write(const std::vector<float>& val, const std::string& id) = 0;
		virtual void write(const std::vector<double>& val, const std::string& id) = 0;
		virtual void write(const std::vector<std::string>& val, const std::string& id) = 0;


		// now for the complex types, these needs to implement save/load functionality
		/**
		 * @brief generic write method. This method will write any objects that are either derived
		 * from the rw::common::Serializable class or where overloaded read() write() methods exists.
		 * @param object
		 * @param id
		 */
		template<class T>
		void write(const T& object, const std::string& id){
			// the data method must have an implementation of load/save and if not then we try the generic write
			// method which could provide a solution by the implementation itself
			writeImpl(object, id);

		}

	private:
	    template<class T>
	    void writeImpl(T& object, const std::string& id, typename boost::enable_if_c<boost::is_base_of<Serializable, T>::value, T>::type* def=NULL){
			object.write(*this, id);
	    }

/*
	    template<class T>
	    void writeImpl(T& object, const std::string& id, typename boost::enable_if_c<boost::is_pointer<T>::value, T>::type* def=NULL){
			write((uint64_t)object, id);
	    }
*/

	    //template<class T>
	    //void writeImpl(T& object, const std::string& id, typename boost::enable_if_c<boost::is_pointer<T>::value, T>::type* def=NULL){
	    //	BOOST_MPL_ASSERT_MSG(boost::is_pointer<T>::value, "type T cannot be of type reference!");
	    //}

	    template<typename T>
	    void writeImpl(T& object, const std::string& id, typename boost::disable_if_c<boost::is_base_of<Serializable, T>::value, T>::type* def=NULL){
	    	//BOOST_MPL_ASSERT_MSG(boost::is_reference<T>::value, "type T cannot be of type reference!" , (T) );

			//if( boost::is_floating_point<T>::value || boost::is_integral<T>::value){
			//	T* val = new T;
			//	write(*val, id);
			//}

			// try and use overloaded method
			serialization::write(object, *this, id);
	    }


	};

}} // namespace end

#endif
