/*********************************************************************
 * RobWork Version 0.2
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
 * for detailed Actionrmation about these packages.
 *********************************************************************/
#ifndef RW_TASK_TARGET_HPP
#define RW_TASK_TARGET_HPP

/**
 * @file Target.hpp
 */

#include "Link.hpp"
#include "Entity.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/PropertyMap.hpp>

#include <boost/variant.hpp>

namespace rw { namespace task {
	class Link;

	/** @addtogroup task */
    /*@{*/

    /**
     * @brief brief Data structure for target specifications in task trajectories.
     *
	 * TODO: Longer description
     */
	class ToolLocation
	{
	public:
		ToolLocation(
            const rw::math::Transform3D<> &transform,
            rw::kinematics::Frame *frame)
            :
            _transform(transform),
            _frame(frame)
		{
            RW_ASSERT(_frame);
        }

		const rw::math::Transform3D<> &getTransform() const { return _transform; }
		rw::kinematics::Frame& getFrame() const { return *_frame; }

	private:
		rw::math::Transform3D<> _transform;
		rw::kinematics::Frame* _frame;
	};

	class Target : public Entity
	{
		friend class Trajectory;

	public:
        typedef boost::variant<rw::math::Q, ToolLocation> value_type;

		Target(
            const Entity& entity,
            const value_type& value);

		Link* getNext() { return _next; }
		Link* getPrev() { return _prev; }

        value_type& getValue() { return _value; }
		const value_type& getValue() const { return _value; }

		void setNext(Link *next) { _next = next; }
		void setPrev(Link *prev) { _prev = prev; }

	private:
		value_type _value;
		Link* _prev;
        Link* _next;
	};

}} // end namespaces

#endif // end include guard
