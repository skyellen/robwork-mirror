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

#ifndef RWLIBS_TASK_TARGET_HPP
#define RWLIBS_TASK_TARGET_HPP

#include "Entity.hpp"
#include "TypeRepository.hpp"

#include <rw/common/Ptr.hpp>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

namespace rwlibs {
namespace task {



/** @addtogroup task */
/*@{*/



/**
 * @brief Base class for targets
 */
class TargetBase: public Entity
{
public:
    /**
     * @brief Constructs TargetBase with a given type
     * @param targetType [in] Type of the target
     */
    TargetBase(int targetType = -1):
        Entity(EntityType::Target),
        _targetType(targetType)
    {}

    /**
     * @brief Destructor
     */
    virtual ~TargetBase() {};


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

/** @} */

} //end namespace task
} //end namespace rwlibs


#endif // end include guard
