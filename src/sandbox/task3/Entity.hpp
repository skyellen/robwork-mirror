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

/** @addtogroup task3 */
/*@{*/


/**
 * @brief Type of an Entity
 */
class EntityType {
public:
    /** Enumeration of the type */
    enum Type {  Undefined = -1 /** Undefined type */
            , Task = 0 /** The entity is a Task */
            , Motion = 1 /** The entity is a Motion */
            , Action = 2 /** The entity is an Action */
            , Target = 3 /** The entity is a Target */
            , User = 1024 /** User defined entities starts with this index */
            };

    /**
     * @brief Constructs an EntityType object.
     *
     * @param type [in] Type of entity. Default is Undefined
     */
    EntityType(int type = Undefined):
        _type(type)
    {
    }


    /**
     * @brief Cast operator enable implicit conversion to int
     *
     * This operator enables using EntityType in a switch statement.
     */
    operator int() {
        return _type;
    }

private:
    int _type;
};


/**
 * @brief Base class of object inserted into a Task
 */
class Entity
{
public:
    /**
     * @brief Constructs an Entity with a given type,
     *
     * @param type [in] Type of entity
     */
    Entity(EntityType type, const std::string& id = ""):
        _entityType(type),
        _index(-1),
        _id(id)
    {}

    /**
     * @brief Destructor
     */
    virtual ~Entity() {}

    /**
     * @brief Returns reference to rw::common::PropertyMap associated with the Entity
     * @return Reference to PropertyMap
     */
    rw::common::PropertyMap& getPropertyMap() {
        return _properties;
    }

    /**
     * @brief Returns reference to rw::common::PropertyMap associated with the Entity
     * @return Reference to PropertyMap
     */
    const rw::common::PropertyMap& getPropertyMap() const {
        return _properties;
    }

    /**
     * @brief Sets the content of the propertymap
     *
     * Overrides the current propertymap with \b propertymap
     * @param propertymap [in] The propertymap to use
     */
    void setPropertyMap(const rw::common::PropertyMap& propertymap) {
    	_properties = propertymap;
    }

    /**
     * @brief Returns index specifying the position of the Entity in a Task
     *
     * The index may be used to determine when the order of Actions and Motions in a Task.
     * It is the responsibility of the task to generate indices. This index does not necessarily
     * refer to an index in a list of entities.
     *
     * @return The index specifying the relative position of the Entity
     */
    int getIndex() {
        return _index;
    }

    /**
     * @brief Sets the order index of the Entity.
     *
     * This method is primarily used by Task to specify the position of an Entity. Modifying
     * the index does not influence where in a task it is located.
     *
     * @param index [in] The index specifying the order
     */
    void setIndex(int index) {
        _index = index;
    }

    /**
     * @brief Returns the type of Entity.
     * @return Type of the Entity
     */
    virtual EntityType entityType() const {
        return _entityType;
    }

    void setId(const std::string& id) {
    	_id = id;
    }

    const std::string& getId() const {
    	return _id;
    }

    /**
     * @brief Method which can be used to explicitly and safely casting an Entity.
     *
     * If the cast in impossible the method may throws a rw::common::Exception or
     * return NULL if casting to a pointer.
     *
     * Example:
     * \code
     * Action* action = myEntity->cast<Action*>();
     * \endcode
     */
    template <class T>
    T cast() {
        try {
            T entity = dynamic_cast<T>(this);
            return entity;
        } catch (std::bad_cast exp) {}
        RW_THROW("Unable to perform cast");
    }

protected:
    rw::common::PropertyMap _properties;
    EntityType _entityType;
    int _index;
    std::string _id;
};

/**
 * @brief Definition of a rw::common::Ptr to an Entity.
 */
typedef rw::common::Ptr<Entity> EntityPtr;

/** @} */

} //end namespace task
} //end namespace rw

#endif //end include guard
