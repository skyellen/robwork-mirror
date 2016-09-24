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

#ifndef RW_COMMON_INPUTARCHIVE_HPP
#define RW_COMMON_INPUTARCHIVE_HPP

#include "Archive.hpp"

#include "Serializable.hpp"
#include <boost/cstdint.hpp>
#include <boost/type_traits.hpp>

#include <Eigen/Core>

namespace rw {
namespace common {
/**
 * @brief an archive interface for reading from a serialized class.
 */
class InputArchive: public virtual Archive {
public:
	//! @brief constructor
    InputArchive(){};

    /**
     * @brief enter specific scope with id when reading.
     * @param id [in] id of the scope to enter
     * @param idDefault [in] the default scope id
     */
    void readEnterScope(const std::string& id, const std::string& idDefault=""){
    	if(id.empty()){
    		doReadEnterScope(idDefault);
    	} else {
    		doReadEnterScope(id);
    	}
    }

    /**
     * @brief leave specific scope with id when reading.
     * @param id [in] id of the scope to leave
     * @param idDefault [in] the default scope id
     */
    void readLeaveScope(const std::string& id, const std::string& idDefault=""){
    	if(id.empty()){
    		doReadLeaveScope(idDefault);
    	} else {
    		doReadLeaveScope(id);
    	}
    }


    // convienience wrappers for reading primitives
    //! read boolean
    bool readBool(const std::string& id){ bool b; read(b,id); return b;};
    //! read integer
    int readInt(const std::string& id){ int b; read(b,id); return b;};
    //! read unsigned integer
    unsigned int readUInt(const std::string& id){ unsigned int b; read(b,id); return b;};
    //! read 8 bit integer
    boost::int8_t readInt8(const std::string& id){ boost::int8_t b; read(b,id); return b;};
    //! read 8 bit unsigned integer
    boost::uint8_t readUInt8(const std::string& id){ boost::uint8_t b; read(b,id); return b;};
    //! read 64 bit integer
    boost::int64_t readInt64(const std::string& id){ boost::int64_t b; read(b,id); return b;};
    //! read 64 bit unsigned integer
    boost::uint64_t readUInt64(const std::string& id){ boost::uint64_t b; read(b,id); return b;};
    //! read double floating point
    double readDouble(const std::string& id) { double b; read(b,id); return b;};
    //! read string
    std::string readString(const std::string& id) { std::string b; read(b,id); return b;};


    /**
     * @brief Read \b object with identifier \b id from archive.
     * @param object [out] the object from archive.
     * @param id [in] the identifier.
     */
    template<class T>
    void read(T& object, const std::string& id){
    	doRead(object, id);
    }

    /**
     * @copydoc read
     * @param idDefault [in] (optional) default identifier used if \b id is an empty string.
     */
    template<class T>
    void read(T& object, const std::string& id, const std::string& idDefault){
    	if(!id.empty())
    		doRead(object, id);
    	else
    		doRead(object, idDefault);
    }

    /**
     * @brief Stream operator for streaming from archive to object.
     * @param dst [out] the object from archive.
     * @return a reference to the archive for chaining.
     */
    template<class T>
    InputArchive& operator>> (T& dst){
    	read<T>(dst,"");
		return *this;
    }

protected:
    //! @copydoc read
    template<class T>
    void doRead(T& object, const std::string& id){
    	readImpl(object, id);
    }

    // reading primitives to archive
    /**
     * @brief Read value \b val with identifier \b id from archive.
     * @param val [out] the value from archive.
     * @param id [in] the identifier.
     */
    virtual void doRead(bool& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(boost::int8_t& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(boost::uint8_t& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(boost::int16_t& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(boost::uint16_t& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(boost::int32_t& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(boost::uint32_t& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(boost::int64_t& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(boost::uint64_t& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(float& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(double& val, const std::string& id) = 0;
    //! @copydoc(bool&,const std::string&)
    virtual void doRead(std::string& val, const std::string& id) = 0;

    /**
     * @brief Read vector \b val with identifier \b id from archive.
     * @param val [out] the vector from archive.
     * @param id [in] the identifier.
     */
    virtual void doRead(std::vector<bool>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<boost::int8_t>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<boost::uint8_t>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<boost::int16_t>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<boost::uint16_t>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<boost::int32_t>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<boost::uint32_t>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<boost::int64_t>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<boost::uint64_t>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<float>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<double>& val, const std::string& id) = 0;
    //! @copydoc(std::vector<bool>&,const std::string&)
    virtual void doRead(std::vector<std::string>& val, const std::string& id) = 0;

	/**
	 * @brief Read an Eigen matrix from input.
	 * @param val [out] the result.
	 * @param id [in] identifier for the matrix.
	 */
    virtual void doRead(Eigen::MatrixXd& val, const std::string& id) = 0;

	/**
	 * @brief Read an Eigen vector from input.
	 * @param val [out] the result.
	 * @param id [in] identifier for the matrix.
	 */
    virtual void doRead(Eigen::VectorXd& val, const std::string& id) = 0;

    /**
     * @brief Enter a scope.
     * @param id [in] identifier for the scope.
     */
    virtual void doReadEnterScope(const std::string& id) = 0;

    /**
     * @brief Leave a scope.
     * @param id [in] identifier for the scope.
     */
    virtual void doReadLeaveScope(const std::string& id) = 0;

private:
    template<class T>
    void readImpl(T& object, const std::string& id, typename boost::enable_if_c<boost::is_base_of<Serializable, T>::value, T>::type* def=NULL){
    	object.read(*this, id);
    }

    template<class T>
    void readImpl(T& object, const std::string& id, typename boost::disable_if_c<boost::is_base_of<Serializable, T>::value, T>::type* def=NULL){
    	// first test if T is any of the primitives
    	if( boost::is_const<T>::value ){
    		RW_THROW("type T cannot be of type const!");
    	} else if( boost::is_reference<T>::value ){
    		RW_THROW("type T cannot be of type reference!");
    	} else if( boost::is_floating_point<T>::value || boost::is_integral<T>::value){
    		read(object,id);
    	} else {
    		serialization::read(object, *this, id);
    	}

    }



};
}} //namespace end
#endif
