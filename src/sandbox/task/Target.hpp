/*
 * Target.hpp
 *
 *  Created on: Jan 29, 2009
 *      Author: lpe
 */

#ifndef RW_TASK_TARGET_HPP
#define RW_TASK_TARGET_HPP

#include "Entity.hpp"

#include <rw/common/Ptr.hpp>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw {
namespace task2 {


class Target: public Entity
{
public:
    Target(int targetType):
        _targetType(targetType)
    {}

    virtual ~Target() {};

    virtual int type() const {
        return Entity::TARGET;
    }

    int targetType() {
        return _targetType;
    }

    static const int Q_TYPE = 0;
    static const int TRANSFORM_TYPE = 1;
    static const int USER = 1024;
private:
    int _targetType;
};

typedef rw::common::Ptr<Target> TargetPtr;


template <class T, int target_type>
class TemplateTarget: public Target
{
public:
    TemplateTarget(const T& value):
        Target(target_type),
        _value(value)
    {
    }

    T& get() { return _value; };
    const T& get() const { return _value; };

private:
    T _value;
};

//class TTarget: public TemplateTarget<rw::math::Q, Target::Q_TYPE>
typedef TemplateTarget<rw::math::Q, Target::Q_TYPE> QTarget;
typedef TemplateTarget<rw::math::Transform3D<>, Target::TRANSFORM_TYPE> TTarget;
//typedef TemplateTarget<std::pair<rw::math::Transform3D<>, rw::math::Q>, Target::TQ_TYPE> TQTarget;

typedef rw::common::Ptr<QTarget> QTargetPtr;
typedef rw::common::Ptr<TTarget> TTargetPtr;
//typedef rw::common::Ptr<TQTarget> TQTargetPtr;

} //end namespace task
} //end namespace rw


#endif // end include guard
