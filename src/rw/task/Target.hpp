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

#include "Property.hpp"

#include "Link.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>

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

	class Target
	{
		friend class Trajectory;

	public:
		Target(
            const boost::variant<rw::math::Q, ToolLocation>& value,
            const std::string& name = "");

		bool isQ() const { return _value.type() == typeid(rw::math::Q); }
		bool isToolLocation() const { return _value.type() == typeid(ToolLocation); }

		const ToolLocation& getToolLocation() const;
		const rw::math::Q& getQ() const;

		const std::string& getName() const { return _name; }

		Property &Properties() { return _properties; };

		Link *next() { return _next; }
		Link *prev() { return _prev; }

        // We give direct access.
        typedef boost::variant<rw::math::Q, ToolLocation> value_type;
        value_type& getValue() { return _value; }
		const value_type& getValue() const { return _value; }

		void setData(const Target &target);

		void setNext(Link *next) { _next = next; }
		void setPrev(Link *prev) { _prev = prev; }
	private:
		value_type _value;
		std::string _name;
		Property _properties;
		Link* _prev;
        Link* _next;
	};

}} // end namespaces

#endif // end include guard
