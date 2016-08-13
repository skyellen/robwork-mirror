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



#ifndef RWLIBS_TASK_TYPEREPOSITORY_HPP
#define RWLIBS_TASK_TYPEREPOSITORY_HPP

#include <rw/common/macros.hpp>
//#include <tr1/unordered_map>
#include <map>
#include <string>
#include <typeinfo>

namespace rwlibs {
namespace task {


    /** @addtogroup task */
    /*@{*/

/**
 * @brief Class defining general types
 */
class Type {
public:
    /**
     * @brief Construct a type object.
     *
     * If no \b id is specified if constructs a Type object with type=Undefined
     *
     * @param id [in] int identifier of the type
     */
    Type(int id = -1):
        _id(id)
    {
    }

    /**
     * Predefined ids
     */
    enum Ids { Undefined = -1 /** Undefined type */
        , Q = 0 /** rw::math::Q type */
        , Transform3D /**rw::math::Transform3D<> type */
        , User = 1024 /**User defined types starts with this */
        };

    /**
     * @brief operator enable implicit cast to int.
     *
     * This operator enables using Type in switch statements
     */
    operator int () {
        return _id;
    }

private:
    int _id;
};

/**
 * @brief The TypeRepository provides a repository in which types can be mapped to Type objects
 *
 * Only one TypeRepository is allowed, hence it is implemented with a singleton pattern.
 *
 * Notice that problems might occur if trying to use the TypeRepository with user defined type across
 * dynamic linked libraries.
 */
class TypeRepository
{
public:

    virtual ~TypeRepository() {};

    /**
     * @brief Adds a new Type to the repository for the template type T
     *
     * If the type already exists the Type object associated are just returned.
     *
     * @return Type object associated to the type T
     */
    template <class T>
    Type add() {
        std::string name = typeid(T).name();
        TypeMap::iterator it = _typeMap.find(name);

        if (it == _typeMap.end()) {
            it = _typeMap.insert(TypeMap::value_type(name, _next++)).first;
        }

        return (*it).second;
    }

    /**
     * @brief Tests whether the template type T exists in the repository
     */
    template <class T>
    bool has() {
        return _typeMap.find(typeid(T).name()) != _typeMap.end();
    }

    /**
     * @brief Returns the Type associated to the template type T
     *
     * With the get method is is possible to specify whether to add the type
     * if it does not exists or whether to throw a rw::common::Exception. If the
     * type if not defined and no exception is throw it returns Undefined.
     */
    template <class T>
    Type get(bool addIfNotExisting = false, bool throwException = true) {
        TypeMap::const_iterator it = _typeMap.find(typeid(T).name());
        if (it != _typeMap.end())
            return (*it).second;
        if (addIfNotExisting)
            return add<T>();
        if (throwException)
            RW_THROW("Type does not exists in TypeRepository");
        return Type::Undefined;

    }

    /**
     * @brief Returns the global instance of the TypeRepository
     * @return Reference to the global TypeRepository
     */
    static TypeRepository& instance() {
        static TypeRepository repository;
        return repository;
    }



private:
    //typedef std::tr1::unordered_map<std::string, Type> TypeMap;
    typedef std::map<std::string, Type> TypeMap;

    TypeMap _typeMap;
    int _next;
    TypeRepository();
};

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
