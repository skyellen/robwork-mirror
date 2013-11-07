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

namespace rw {
namespace common {
/**
 * @brief an archive interface for reading from a serialized class.
 */
class InputArchive: public Archive {
public:
	//! @brief constructor
    InputArchive(){};

    //! @brief open an inputstream for reading
    virtual void open(std::istream& ifs) = 0;

    // utils to handle arrays
    virtual void readEnterScope(const std::string& id) = 0;
    virtual void readLeaveScope(const std::string& id) = 0;

    // reading primitives to archive
    virtual void read(bool& val, const std::string& id) = 0;
    virtual void read(boost::int8_t& val, const std::string& id) = 0;
    virtual void read(boost::uint8_t& val, const std::string& id) = 0;
    virtual void read(boost::int16_t& val, const std::string& id) = 0;
    virtual void read(boost::uint16_t& val, const std::string& id) = 0;
    virtual void read(boost::int32_t& val, const std::string& id) = 0;
    virtual void read(boost::uint32_t& val, const std::string& id) = 0;
    virtual void read(boost::int64_t& val, const std::string& id) = 0;
    virtual void read(boost::uint64_t& val, const std::string& id) = 0;
    virtual void read(float& val, const std::string& id) = 0;
    virtual void read(double& val, const std::string& id) = 0;
    virtual void read(std::string& val, const std::string& id) = 0;

    virtual void read(std::vector<bool>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::int8_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::uint8_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::int16_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::uint16_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::int32_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::uint32_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::int64_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::uint64_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<float>& val, const std::string& id) = 0;
    virtual void read(std::vector<double>& val, const std::string& id) = 0;
    virtual void read(std::vector<std::string>& val, const std::string& id) = 0;




    // convienience wrappers for reading primitives
    bool readBool(const std::string& id){ bool b; read(b,id); return b;};
    int readInt(const std::string& id){ int b; read(b,id); return b;};
    unsigned int readUInt(const std::string& id){ unsigned int b; read(b,id); return b;};

    boost::int8_t readInt8(const std::string& id){ boost::int8_t b; read(b,id); return b;};
    boost::uint8_t readUInt8(const std::string& id){ boost::uint8_t b; read(b,id); return b;};

    boost::int64_t readInt64(const std::string& id){ boost::int64_t b; read(b,id); return b;};
    boost::uint64_t readUInt64(const std::string& id){ boost::uint64_t b; read(b,id); return b;};
    double readDouble(const std::string& id) { double b; read(b,id); return b;};
    std::string readString(const std::string& id) { std::string b; read(b,id); return b;};


    //
    template<class T>
    void read(T& object, const std::string& id){
    	readImpl(object, id);
    }

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
    		serialization::read<T>(object, *this, id);
    	}

    }



};
}} //namespace end
#endif
