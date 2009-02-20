/*
 * TypeRepository.hpp
 *
 *  Created on: Feb 4, 2009
 *      Author: lpe
 */

#ifndef RW_TASK_TYPEREPOSITORY_HPP
#define RW_TASK_TYPEREPOSITORY_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/macros.hpp>
//#include <tr1/unordered_map>
#include <map>
#include <string>
#include <typeinfo>

namespace rw {
namespace task3 {

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
     *
     */
    enum Ids { Undefined = -1, Q = 0, Transform3D, User = 1024};

    operator int () {
        return _id;
    }

private:
    int _id;
};

class TypeRepository
{
public:
    virtual ~TypeRepository() {};

    template <class T>
    Type add() {
        std::string name = typeid(T).name();
        TypeMap::iterator it = _typeMap.find(name);

        if (it == _typeMap.end()) {
            //std::cout<<"Type Map["<<name<<"] = "<<_next<<std::endl;
            it = _typeMap.insert(TypeMap::value_type(name, _next++)).first;
        } else {
            //std::cout<<"Already got "<<name<<" as "<<(*it).second<<std::endl;
        }

        return (*it).second;
    }

    template <class T>
    bool has() {
        return _typeMap.find(typeid(T).name()) != _typeMap.end();
    }

    template <class T>
    Type get(bool addIfNotExisting = false, bool throwException = true) {
        TypeMap::const_iterator it = _typeMap.find(typeid(T).name());
        //std::cout<<"Asks for "<<typeid(T).name()<<std::endl;
        if (it != _typeMap.end())
            return (*it).second;
        if (addIfNotExisting)
            return add<T>();
        if (throwException)
            RW_THROW("Type does not exists in TypeRepository");
        return -1;

    }

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

} //end namespace task3
} //end namespace rw

#endif //end include guard
