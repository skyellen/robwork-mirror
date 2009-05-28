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
namespace task2 {

class Motion: public Entity
{
public:
    Motion(int motionType):
        _motionType(motionType)
    {
    }

    virtual ~Motion() {};

    static const int P2P = 0;
    static const int Linear = 1;
    static const int Circular = 2;
    static const int User = 1024;

    int motionType() {
        return _motionType;
    }

    virtual int type() const {
        return MOTION;
    }

private:
    int _motionType;
};

typedef rw::common::Ptr<Motion> MotionPtr;

//typedef rw::common::Ptr<Motion> MotionPtr;


class P2PMotion: public Motion {
public:
    P2PMotion(TargetPtr start, TargetPtr end):
        Motion(P2P),
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

typedef rw::common::Ptr<P2PMotion> P2PMotionPtr;


class LinearMotion: public Motion {
public:
    LinearMotion(TargetPtr start, TargetPtr end):
        Motion(LINEAR),
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

typedef rw::common::Ptr<LinearMotion> LinearMotionPtr;

class CircularMotion: public Motion {
public:
    CircularMotion(TargetPtr start, TargetPtr mid, TargetPtr end):
        Motion(CIRCULAR),
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

typedef rw::common::Ptr<CircularMotion> CircularMotionPtr;

} //end namespace task
} //end namespace rw

#endif //end include guard
