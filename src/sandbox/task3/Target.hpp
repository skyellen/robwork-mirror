/*
 * Target.hpp
 *
 *  Created on: Jan 29, 2009
 *      Author: lpe
 */

#ifndef RW_TASK_TARGET_HPP
#define RW_TASK_TARGET_HPP

#include "Entity.hpp"
#include "TypeRepository.hpp"

#include <rw/common/Ptr.hpp>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw {
namespace task3 {

/**
 * @brief Base class for targets
 */
class TargetBase: public Entity
{
public:
    /**
     * @brief Constructs TargetBase with a given type
     * @paran targetType [in] Type of the target
     */
    TargetBase(int targetType = -1):
        Entity(EntityType::Target),
        _targetType(targetType)
    {}

    /**
     * @brief Destructor
     */
    virtual ~TargetBase() {};

  /*  virtual int type() const {
        return Entity::TargetId;
    }
*/
    /**
     * @brief Returns the type of target
     */
    Type targetType() {
        return _targetType;
    }

    /**
     * @brief Returns the value of the target.
     *
     * The user need to provide the type as template argument.
     *
     * May throw a rw::common::Exception if type conversion is invalid.
     * @return Value of target
     */
    template <class T>
    T& getValue();

protected:
    Type _targetType;
};





/**
 * Definition of pointer to target
 */
typedef rw::common::Ptr<TargetBase> TargetPtr;


/**
 * @brief Template class implementing Target
 */
template <class T>
class Target: public TargetBase
{
public:
    /**
     * @brief Construct Target with value \b value.
     * @param value [in] Value of target
     */
    Target(const T& value):
        _value(value)
    {
        _targetType = TypeRepository::instance().get<T>(true);
    }

    /**
     * @brief Returns the value of the target
     * @return Value of target
     */
    T& get() { return _value; };

    /**
     * @brief Returns the value of the target
     * @return Value of target
     */
    const T& get() const { return _value; };

private:
    T _value;
};

/**
 * Definition of Target with type rw::math::Q
 */
typedef Target<rw::math::Q> QTarget;

/**
 * Definition of Target with type rw::math::Transform3D
 */
typedef Target<rw::math::Transform3D<> > CartesianTarget;

/**
 * Definition of rw::common::Ptr to QTarget
 */
typedef rw::common::Ptr<QTarget> QTargetPtr;

/**
 * Definition of rw::common::Ptr to CartesianTarget
 */
typedef rw::common::Ptr<CartesianTarget> CartesianTargetPtr;


template <class T>
T& TargetBase::getValue()
{
    Target<T>* target = dynamic_cast<Target<T>*>(this);
    if (target != NULL) {
        return target->get();
    }
    RW_THROW("Unable to convert target to specified type");
}

} //end namespace task
} //end namespace rw


#endif // end include guard
