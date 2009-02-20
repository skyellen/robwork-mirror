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
namespace task3 {

/** @addtogroup task3 */
/*@{*/


/**
 * @brief Specification of Action Type.
 */
class MotionType {
public:
    /** Enumeration for different Motion types */
    enum {  Undefined = -1 /** Undefined Motion */
            , P2P = 0 /** A point-to-point Motion */
            , Linear=1 /** A linear motion */
            , Circular=2 /** A circular motion */
            , User = 1024 /** User defined motions starts with this index */
            };

    /**
     * @brief Constructs a MotionType object.
     *
     * @param type [in] Type of motion. Default is Undefined
     */
    MotionType(int type = Undefined):
        _type(type)
    {
    }

    /**
     * @brief Cast operator enable implicit conversion to int
     *
     * This operator enables using MotionType in a switch statement.
     */
    operator int () {
        return _type;
    }

private:
    int _type;
};

/**
 * @brief Base class for motions providing common interface.
 */
class MotionBase: public Entity
{
public:
    /**
     * @brief Constructs motion
     * @param motionType [in] Type of the motion
     */
    MotionBase(MotionType motionType):
        Entity(EntityType::Motion),
        _motionType(motionType)
    {
    }

    /**
     * @brief Destructor
     */
    virtual ~MotionBase() {};

    /**
     * @brief Returns the type of the motion
     * @return Type of motion
     */
    MotionType motionType() {
        return _motionType;
    }

   /* virtual int type() const {
        return MotionId;
    }*/


private:
    int _motionType;
};

/**
 * Typedef of rw::common::Ptr to a Motion.
 */
typedef rw::common::Ptr<MotionBase> MotionPtr;

/**
 * @brief Specified the template based interface of a motion
 *
 * This class is abstract.
 *
 */
template <class T>
class Motion: public MotionBase {
public:
    /**
     * Convenient typedef of pointer to target of type T
     */
    typedef rw::common::Ptr<Target<T> > TargetPtr;

    /**
     * @brief Returns value of the start target
     * @return Reference to value of start target
     */
    virtual const T& start() = 0;

    /**
     * @brief Returns value of the end target
     * @return Reference to value of end target
     */
    virtual const T& end() = 0;

    /**
     * @brief Returns the start target
     * @return The start target
     */
    virtual TargetPtr startTarget() = 0;

    /**
     * @brief Returns the end target
     * @return The start target
     */
    virtual TargetPtr endTarget() = 0;

protected:

    /**
     * @brief Protected constructor
     */
    Motion(MotionType motion_type):
        MotionBase(motion_type)
    {
        TypeRepository::instance().add<T>();
    }


};

/**
 * Definition of motion using rw::math::Q
 */
typedef Motion<rw::math::Q> QMotion;

/**
 * Definition of motion using rw::math::Transform3D
 */
typedef Motion<rw::math::Transform3D<> > CartesianMotion;

/**
 * Pointer to motion using rw::math::Q
 */
typedef rw::common::Ptr<QMotion> QMotionPtr;

/**
 * Pointer to motion using rw::math::Q
 */
typedef rw::common::Ptr<CartesianMotion> CartesianMotionPtr;

/**
 * @brief Class describing point to point motions.
 *
 * A point to point motion is linear in joint space and will generally
 * be the fastest way of moving between two configurations.
 */
template <class T>
class P2PMotion: public Motion<T>
{
public:
    /**
     * Definition of target for convenience.
     */
    typedef rw::common::Ptr<Target<T> > TargetPtr;

    /**
     * @brief Constructs P2PMotion between \b start and \b end
     *
     * @param start [in] start of the motion
     * @param end [in] end of the motion
     */
    P2PMotion(TargetPtr start, TargetPtr end):
        Motion<T>(MotionType::P2P),
        _start(start),
        _end(end)
    {}

    /**
     * @copydoc Motion::start
     */
    const T& start() {
        return _start->get();
    }

    /**
     * @copydoc Motion::end
     */
    const T& end() {
        return _end->get();
    }

    /**
     * @copydoc Motion::startTarget
     */
    TargetPtr startTarget() {
        return _start;
    }

    /**
     * @copydoc Motion::endTarget
     */
    TargetPtr endTarget() {
        return _end;
    }


private:
    TargetPtr _start;
    TargetPtr _end;
};

/**
 * Definition of rw::math::Q based point to point motion
 */
typedef P2PMotion<rw::math::Q> QP2PMotion;

/**
 * Definition of rw::math::Transform3D based point to point motion
 */
typedef P2PMotion<rw::math::Transform3D<> > CartesianP2PMotion;

/**
 * Definition of rw::math::Ptr to QP2PMotion
 */
typedef rw::common::Ptr<QP2PMotion> QP2PMotionPtr;

/**
 * Definition of rw::math::Ptr to CartesianP2PMotion
 */
typedef rw::common::Ptr<CartesianP2PMotion> CartesianP2PMotionPtr;

/**
 * @brief Class describing linear motions.
 *
 * A linear motion is defined as a motion linear in Cartesian space.
 * It is the users responsibility to decide of to interpolate the orientation.
 */
template <class T>
class LinearMotion: public Motion<T> {
public:
    /**
     * Definition of target for convenience.
     */
    typedef rw::common::Ptr<Target<T> > TargetPtr;

    /**
     * @brief Construct LinearMotion from \b start to \b end
     * @param start [in] start of motion
     * @param end [in] end of motion
     */
    LinearMotion(TargetPtr start, TargetPtr end):
        Motion<T>(MotionType::Linear),
        _start(start),
        _end(end)
    {}

    /**
     * @copydoc Motion::start
     */
    const T& start() {
        return _start->get();
    }

    /**
     * @copydoc Motion::end
     */
    const T& end() {
        return _end->get();
    }

    /**
     * @copydoc Motion::startTarget
     */
     TargetPtr startTarget() {
         return _start;
     }

     /**
      * @copydoc Motion::endTarget
      */
     TargetPtr endTarget() {
         return _end;
     }
private:
    TargetPtr _start;
    TargetPtr _end;
};

/**
 * Definition of rw::math::Q based linear motion.
 */
typedef LinearMotion<rw::math::Q> QLinearMotion;

/**
 * Definition of rw::math::Transform3D based linear motion.
 */
typedef LinearMotion<rw::math::Transform3D<> > CartesianLinearMotion;

/**
 * Definition of rw::common::Ptr to QLinearMotion
 */
typedef rw::common::Ptr<QLinearMotion> QLinearMotionPtr;

/**
 * Definition of rw::common::Ptr to CartesianLinearMotion
 */
typedef rw::common::Ptr<CartesianLinearMotion> CartesianLinearMotionPtr;

/**
 * @brief Class describing circular motions.
 *
 * Circular motions are generally defined in Cartesian space.
 * It is the reponsibility of the user to decide how to interpolate the rotation.
 */
template <class T>
class CircularMotion: public Motion<T> {
public:
    /**
     * Definition of target for convenience.
     */
    typedef rw::common::Ptr<Target<T> > TargetPtr;

    /**
     * @brief Construct a CircularMotion starting in \b start, going through \b mid and ending in \b end
     * @param start [in] motion start target.
     * @param mid [in] target to reach somewhere between \b start and \b end.
     * @param end [in] motion end target
     */
    CircularMotion(TargetPtr start, TargetPtr mid, TargetPtr end):
        Motion<T>(MotionType::Circular),
        _start(start),
        _mid(mid),
        _end(end)
    {}

    /**
     * @copydoc Motion::start
     */
    const T& start() {
        return _start->get();
    }

    /**
     * @brief Returns the value of the mid point target
     * @return Value of target
     */
    const T& mid() {
        return _mid->get();
    }

    /**
     * @copydoc Motion::end
     */
    const T& end() {
        return _end->get();
    }

    /**
     * @copydoc Motion::startTarget
     */
    TargetPtr startTarget() {
        return _start;
    }

    /**
     * @brief Returns the mid point target
     * @return Mid point target
     */
    TargetPtr midTarget() {
        return _mid;
    }

    /**
     * @copydoc Motion::endTarget
     */
    TargetPtr endTarget() {
        return _end;
    }

private:
    TargetPtr _start;
    TargetPtr _mid;
    TargetPtr _end;
};

/**
 * @brief Definition of circular motion with rw::math::Transform type
 */
typedef CircularMotion<rw::math::Transform3D<> > CartesianCircularMotion;

/**
 * @brief Pointer to CartesianCircularMotion
 */
typedef rw::common::Ptr<CartesianCircularMotion> CartesianCircularMotionPtr;

/** @} */

} //end namespace task
} //end namespace rw

#endif //end include guard
