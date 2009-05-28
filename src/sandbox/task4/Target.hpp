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
namespace task4 {

/*
class TargetBase: public Entity
{
public:
    TargetBase(int targetType = -1):
        Entity(EntityType::Target),
        _targetType(targetType)
    {}

    virtual ~TargetBase() {};


    Type targetType() {
        return _targetType;
    }

    template <class T>
    T& get();

protected:
    Type _targetType;
};

typedef rw::common::Ptr<TargetBase> TargetPtr;



template <class T>
class Target: public TargetBase
{
public:

    Target(const T& value):
        _value(value)
    {
        _targetType = TypeRepository::instance().get<T>();
    }

    T& getValue() { return _value; };
    const T& getValue() const { return _value; };

private:
    T _value;
};

typedef Target<rw::math::Q> QTarget;
typedef Target<rw::math::Transform3D<> > CartesianTarget;

typedef rw::common::Ptr<QTarget> QTargetPtr;
typedef rw::common::Ptr<CartesianTarget> CartesianTargetPtr;

template <class T>
T& TargetBase::get()
{
    Target<T>* target = dynamic_cast<Target<T>*>(this);
    if (target != NULL) {
        return target->getValue();
    }
    RW_THROW("Unable to convert target to specified type");
}
*/


} //end namespace task
} //end namespace rw


#endif // end include guard
