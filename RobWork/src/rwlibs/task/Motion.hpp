/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RWLIBS_TASK_MOTION_HPP
#define RWLIBS_TASK_MOTION_HPP

#include "Entity.hpp"
#include "Target.hpp"

#include <rw/common/Ptr.hpp>

#include <boost/foreach.hpp>

namespace rwlibs {
namespace task {

/** @addtogroup task */
/*@{*/


/**
 * @brief Specification of Action Type.
 */
class MotionType {
public:
    /** Enumeration for different Motion types */
    enum Type {  Undefined = -1 /** Undefined Motion */
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
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<MotionBase> Ptr;
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
 * @brief Specified the template based interface of a motion
 *
 * This class is abstract.
 *
 */
template <class T>
class Motion: public MotionBase {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<Motion<T> > Ptr;

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

    /**
     * @brief Make a copy of the motion.
     * @param newTargets [in] a vector of targets.
     * @return new identical motion.
     */
    virtual rw::common::Ptr<Motion<T> > clone(const std::vector<TargetPtr>& newTargets) = 0;

    //! @brief Do reverse motion.
    virtual void reverse() = 0;
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
 * @brief Class describing point to point motions.
 *
 * A point to point motion is linear in joint space and will generally
 * be the fastest way of moving between two configurations.
 */
template <class T>
class P2PMotion: public Motion<T>
{
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<P2PMotion<T> > Ptr;

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

    //! @copydoc Motion::clone
    virtual rw::common::Ptr<Motion<T> > clone(const std::vector<TargetPtr>& newTargets) {
    	TargetPtr start;
    	TargetPtr end;
    	BOOST_FOREACH(TargetPtr target, newTargets) {
    		if (target->getIndex() == startTarget()->getIndex())
    			start = target;
    		if (target->getIndex() == endTarget()->getIndex())
    			end = target;
    	}
    	rw::common::Ptr<Motion<T> > result = rw::common::ownedPtr(new P2PMotion<T>(start, end));
    	result->setPropertyMap(this->getPropertyMap());
    	result->setIndex(this->getIndex());
    	result->setId(this->getId());
    	return result;
    }

    //! @copydoc Motion::reverse
    virtual void reverse() {
    	std::swap(_start, _end);
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
 * @brief Class describing linear motions.
 *
 * A linear motion is defined as a motion linear in Cartesian space.
 * It is the users responsibility to decide of to interpolate the orientation.
 */
template <class T>
class LinearMotion: public Motion<T> {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<LinearMotion<T> > Ptr;

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

     //! @copydoc Motion::clone
     virtual rw::common::Ptr<Motion<T> > clone(const std::vector<TargetPtr>& newTargets) {
     	TargetPtr start;
     	TargetPtr end;
     	BOOST_FOREACH(TargetPtr target, newTargets) {
     		if (target->getIndex() == startTarget()->getIndex())
     			start = target;
     		if (target->getIndex() == endTarget()->getIndex())
     			end = target;
     	}
     	return rw::common::ownedPtr(new LinearMotion<T>(start, end));
     }

     //! @copydoc Motion::reverse
     virtual void reverse() {
     	std::swap(_start, _end);
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
 * @brief Class describing circular motions.
 *
 * Circular motions are generally defined in Cartesian space.
 * It is the reponsibility of the user to decide how to interpolate the rotation.
 */
template <class T>
class CircularMotion: public Motion<T> {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<CircularMotion<T> > Ptr;

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

    //! @copydoc Motion::clone
    virtual rw::common::Ptr<Motion<T> > clone(const std::vector<TargetPtr>& newTargets) {
		TargetPtr start, mid, end;
		BOOST_FOREACH(TargetPtr target, newTargets) {
			if (target->getIndex() == _start->getIndex())
				start = target;
			if (target->getIndex() == _mid->getIndex())
				mid = target;
			if (target->getIndex() == _end->getIndex())
				end = target;
		}
		return rw::common::ownedPtr(new CircularMotion(start, mid, end));
	}

    //! @copydoc Motion::reverse
    virtual void reverse() {
    	std::swap(_start, _end);
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

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
