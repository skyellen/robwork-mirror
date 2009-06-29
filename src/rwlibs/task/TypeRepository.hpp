/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/


#ifndef RWLIBS_TASK_TYPEREPOSITORY_HPP
#define RWLIBS_TASK_TYPEREPOSITORY_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
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
     * If no \id is specified if constructs a Type object with type=Undefined
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
        if (_repository == NULL)
            _repository = new TypeRepository();
        return *_repository;
    }



private:
    //typedef std::tr1::unordered_map<std::string, Type> TypeMap;
    typedef std::map<std::string, Type> TypeMap;

    TypeMap _typeMap;
    int _next;
    TypeRepository() {
        _typeMap[typeid(rw::math::Q).name()] = Type::Q;
        _typeMap[typeid(rw::math::Transform3D<>).name()] = Type::Transform3D;
        _next = Type::User;

    }

    static TypeRepository* _repository;


};

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
