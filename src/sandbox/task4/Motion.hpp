/*
 * Motion.hpp
 *
 *  Created on: Jan 29, 2009
 *      Author: lpe
 */

#ifndef RW_TASK_MOTION_HPP
#define RW_TASK_MOTION_HPP

#include "Entity.hpp"
#include "Target.hpp"

#include <rw/common/Ptr.hpp>

namespace rw {
namespace task4 {

class MotionType {
public:
    enum { Undefined = -1, P2P = 0, Linear=1, Circular=2, User = 1024};
    MotionType(int type = Undefined):
        _type(type)
    {
    }

    operator int () {
        return _type;
    }

private:
    int _type;
};


class MotionBase: public Entity
{
public:
    MotionBase(MotionType motionType):
        Entity(EntityType::Motion),
        _motionType(motionType)
    {
    }

    virtual ~MotionBase() {};


    MotionType motionType() {
        return _motionType;
    }

   /* virtual int type() const {
        return MotionId;
    }*/


private:
    int _motionType;
};

typedef rw::common::Ptr<MotionBase> MotionPtr;

template <class T>
class Motion: public MotionBase {
protected:
    Motion(MotionType motion_type):
    MotionBase(motion_type)
    {
    }

};

typedef Motion<rw::math::Q> QMotion;
typedef Motion<rw::math::Transform3D<> > CartesianMotion;
typedef rw::common::Ptr<QMotion> QMotionPtr;
typedef rw::common::Ptr<CartesianMotion> CartesianMotionPtr;

template <class T>
class P2PMotion: public Motion<T>
{
public:
    typedef rw::common::Ptr<Target<T> > TargetPtr;
    P2PMotion(T* start, TargetPtr end):
        Motion<T>(MotionType::P2P),
        _start(start),
        _end(end)
    {}

    T& start() {
        return *_start;
    }

    T& end() {
        return *_end;
    }

private:
    T* _start;
    T* _end;
};

typedef P2PMotion<rw::math::Q> QP2PMotion;
typedef P2PMotion<rw::math::Transform3D<> > CartesianP2PMotion;
typedef rw::common::Ptr<QP2PMotion> QP2PMotionPtr;
typedef rw::common::Ptr<CartesianP2PMotion> CartesianP2PMotionPtr;

template <class T>
class LinearMotion: public Motion<T> {
public:
    typedef rw::common::Ptr<Target<T> > TargetPtr;

    LinearMotion(TargetPtr start, TargetPtr end):
        Motion<T>(MotionType::Linear),
        _start(start),
        _end(end)
    {}

    TargetPtr start() {
        return _start;
    }

    TargetPtr end() {
        return _end;
    }
private:
    TargetPtr _start;
    TargetPtr _end;
};

typedef LinearMotion<rw::math::Q> QLinearMotion;
typedef LinearMotion<rw::math::Transform3D<> > CartesianLinearMotion;
typedef rw::common::Ptr<QLinearMotion> QLinearMotionPtr;
typedef rw::common::Ptr<CartesianLinearMotion> CartesianLinearMotionPtr;


template <class T>
class CircularMotion: public Motion<T> {
public:
    typedef rw::common::Ptr<Target<T> > TargetPtr;
    CircularMotion(TargetPtr start, TargetPtr mid, TargetPtr end):
        Motion<T>(MotionType::Circular),
        _start(start),
        _mid(mid),
        _end(end)
    {}

    TargetPtr start() {
        return _start;
    }

    TargetPtr mid() {
        return _mid;
    }

    TargetPtr end() {
        return _end;
    }
private:
    TargetPtr _start;
    TargetPtr _mid;
    TargetPtr _end;
};

typedef CircularMotion<rw::math::Transform3D<> > CartesianCircularMotion;
typedef rw::common::Ptr<CartesianCircularMotion> CartesianCircularMotionPtr;

} //end namespace task
} //end namespace rw

#endif //end include guard
