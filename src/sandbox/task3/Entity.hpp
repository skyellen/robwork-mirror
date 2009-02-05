/*
 * Entity.hpp
 *
 *  Created on: Jan 29, 2009
 *      Author: lpe
 */

#ifndef RW_TASK_ENTITY_HPP
#define RW_TASK_ENTITY_HPP

#include <rw/common/PropertyMap.hpp>

namespace rw {
namespace task3 {


class EntityType {
public:
    enum { Undefined = -1, Task = 0, Motion = 1, Action = 2, Target = 3, User = 1024};

    EntityType(int type = Undefined):
        _type(type)
    {
    }

    operator int() {
        return _type;
    }

private:
    int _type;
};


class Entity
{
public:
    Entity(EntityType type):
        _entityType(type),
        _orderIndex(-1)
    {}

    virtual ~Entity() {}

    rw::common::PropertyMap& getPropertyMap() {
        return _properties;
    }

    const rw::common::PropertyMap& getPropertyMap() const {
        return _properties;
    }

    int getOrderIndex() {
        return _orderIndex;
    }

    void setOrderIndex(int index) {
        _orderIndex = index;
    }


    virtual EntityType entityType() const {
        return _entityType;
    }

    template <class T>
    T cast(bool throwException = false) {
        try {
            T entity = dynamic_cast<T>(this);
            return entity;
        } catch (std::bad_cast exp) {}
        RW_THROW("Unable to perform cast");
    }

protected:
    rw::common::PropertyMap _properties;
    EntityType _entityType;
    int _orderIndex;
};


typedef rw::common::Ptr<Entity> EntityPtr;

} //end namespace task
} //end namespace rw

#endif //end include guard
