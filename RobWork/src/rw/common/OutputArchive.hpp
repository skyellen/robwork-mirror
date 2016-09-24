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

#include <Eigen/Core>

namespace rw {
namespace common {
	/**
	 * @brief serializable objects can be written to an output archive.
	 *
	 * This class define an interface for serializing data.
	 *
	 */
	class OutputArchive : public virtual Archive {
	public:
		//! @brief destructor
		virtual ~OutputArchive(){};


		// utils to handle arrays
		/**
		 * @brief create a serialized scope in which objects can be written
		 * @param id [in] id of the scope
		 * @param idDefault [in] (optional) default id to use if \b id is an empty string.
		 */
	    void writeEnterScope(const std::string& id, const std::string& idDefault=""){
	    	if(id.empty()){
	    		doWriteEnterScope(idDefault);
	    	} else {
	    		doWriteEnterScope(id);
	    	}
	    }

		/**
		 * @brief leave the current scope
		 * @param id [in] id of the scope
		 * @param idDefault [in] (optional) default id to use if \b id is an empty string.
		 */
	    void writeLeaveScope(const std::string& id, const std::string& idDefault=""){
	    	if(id.empty()){
	    		doWriteLeaveScope(idDefault);
	    	} else {
	    		doWriteLeaveScope(id);
	    	}
	    }


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
			doWrite(object, id);
		}

		/**
		 * @brief Same as write(object,id) however an additional parameter is given which is the default
		 * identifier to use in case id is empty.
		 * @param object [in] object to serialize
		 * @param id [in] identifier
		 * @param id_default [in] default id
		 */
		template<class T>
		void write(const T& object, const std::string& id, const std::string& id_default){
			// the data method must have an implementation of load/save and if not then we try the generic write
			// method which could provide a solution by the implementation itself
			if(!id.empty())
				doWrite(object, id);
			else
				doWrite(object, id_default);
		}

		/**
		 * @brief Output stream operator
		 */
	    template<class T>
	    OutputArchive& operator<< (T& dst){
	    	write<T>(dst,"");
			return *this;
	    }



	protected:


		// writing primitives to archive
	    /**
	     * @brief Write value \b val to archive with identifier \b id.
	     * @param val [in] value to write.
	     * @param id [in] identifier for the value.
	     */
		virtual void doWrite(bool val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(boost::int8_t val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(boost::uint8_t val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(boost::int16_t val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(boost::uint16_t val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(boost::int32_t val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(boost::uint32_t val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(boost::int64_t val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(boost::uint64_t val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(float val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(double val, const std::string& id) = 0;
		//! @copydoc doWrite(bool,const std::string&)
		virtual void doWrite(const std::string& val, const std::string& id) = 0;

	    /**
	     * @brief Write vector \b val to archive with identifier \b id.
	     * @param val [in] vector to write.
	     * @param id [in] identifier for the vector.
	     */
		virtual void doWrite(const std::vector<bool>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<boost::int8_t>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<boost::uint8_t>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<boost::int16_t>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<boost::uint16_t>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<boost::int32_t>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<boost::uint32_t>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<boost::int64_t>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<boost::uint64_t>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<float>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<double>& val, const std::string& id) = 0;
		//! @copydoc doWrite(const std::vector<bool>&,const std::string&)
		virtual void doWrite(const std::vector<std::string>& val, const std::string& id) = 0;

		/**
		 * @brief Write an Eigen matrix to output.
		 * @param val [in] the matrix to output.
		 * @param id [in] identifier for the matrix.
		 */
		virtual void doWrite(const Eigen::MatrixXd& val, const std::string& id) = 0;

		/**
		 * @brief Write an Eigen vector to output.
		 * @param val [in] the vector to output.
		 * @param id [in] identifier for the matrix.
		 */
		virtual void doWrite(const Eigen::VectorXd& val, const std::string& id) = 0;

		/**
		 * @brief handles serialization of an object. The object must either be a primitive type,
		 * inherit from Serializable or there must exist an overloaded method
		 * rw::common::serialization::write(const T& data, class OutputArchive& oarchive, const std::string& id)
		 *
		 * @param object [in] object to be serialized
		 * @param id [in] potential id associated to the object
		 */
		template<class T>
		void doWrite(const T& object, const std::string& id){
			// the data method must have an implementation of load/save and if not then we try the generic write
			// method which could provide a solution by the implementation itself
			writeImpl(object, id);
		}

		/**
		 * @brief Enter a scope.
		 * @param id [in] identifier for the scope.
		 */
		virtual void doWriteEnterScope(const std::string& id) = 0;

		/**
		 * @brief Leave a scope.
		 * @param id [in] identifier for the scope.
		 */
		virtual void doWriteLeaveScope(const std::string& id) = 0;


	private:

		/**
		 * this function should only be called if the object inherits from Serializable
		 */
	    template<class T>
	    void writeImpl(T& object, const std::string& id,
	    		typename boost::enable_if_c<boost::is_base_of<Serializable, T>::value, T>::type* def=NULL)
	    {
			object.write(*this, id);
	    }

	    /**
	     * This function should be called if the object does not inherit from Serializable and if the
	     * object is not a pointer
	     */
	    template<typename T>
	    void writeImpl(T& object, const std::string& id,
	    		typename boost::disable_if_c<boost::is_base_of<Serializable, T>::value, T>::type* def=NULL,
	    		typename boost::disable_if_c<boost::is_pointer<T>::value, T>::type* defptr=NULL)
	    {
	    	//BOOST_MPL_ASSERT_MSG(boost::is_reference<T>::value, "type T cannot be of type reference!" , (T) );

			//if( boost::is_floating_point<T>::value || boost::is_integral<T>::value){
			//	T* val = new T;
			//	write(*val, id);
			//}

			// try and use overloaded method
			serialization::write(object, *this, id);
	    }

	    /**
	     * This function should be called if the object is a pointer type
	     */
	    template<typename T>
	    void writeImpl(T& object, const std::string& id,
	    		typename boost::disable_if_c<boost::is_base_of<Serializable, T>::value, T>::type* def=NULL,
	    		typename boost::enable_if_c<boost::is_pointer<T>::value, T>::type* defptr=NULL)
	    {
			doWrite((boost::uint64_t)object, id);
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


	};

}} // namespace end

#endif
