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
#include <tr1/unordered_map>
#include <string>
#include <typeinfo>

namespace rw {
namespace task3 {

class Type {
public:
    Type(int id = -1):
        _id(id)
    {
    }

    enum Ids { Q = 0, Transform3D};

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

        if (it == _typeMap.end())
            it = _typeMap.insert(TypeMap::value_type(name, _next++)).first;

        return (*it).second;
    }

    template <class T>
    bool has() {
        return _typeMap.find(typeid(T).name()) != _typeMap.end();
    }

    template <class T>
    Type get(bool throwException = true) {
        TypeMap::const_iterator it = _typeMap.find(typeid(T).name());
        if (it != _typeMap.end())
            return (*it).second;
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
    typedef std::tr1::unordered_map<std::string, Type> TypeMap;

    TypeMap _typeMap;
    int _next;
    TypeRepository() {
        _next = 0;
        _typeMap[typeid(rw::math::Q).name()] = Type::Q;
        _typeMap[typeid(rw::math::Transform3D<>).name()] = Type::Transform3D;
    }

    static TypeRepository* _repository;


};

} //end namespace task3
} //end namespace rw

#endif //end include guard
