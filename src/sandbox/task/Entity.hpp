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
namespace task2 {



class Entity
{
public:
    Entity() {}
    virtual ~Entity() {}

    rw::common::PropertyMap& getPropertyMap() {
        return _properties;
    }

    const rw::common::PropertyMap& getPropertyMap() const {
        return _properties;
    }

    static const int TASK = 0;
    static const int MOTION = 1;
    static const int ACTION = 2;
    static const int TARGET = 3;
    static const int USER = 1024;

    virtual int type() const = 0;

protected:
    rw::common::PropertyMap _properties;
};

typedef rw::common::Ptr<Entity> EntityPtr;

} //end namespace task
} //end namespace rw

#endif //end include guard
