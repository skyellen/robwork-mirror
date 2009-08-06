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
 * for detailed Actionrmation about these packages.
 *********************************************************************/

#ifndef RW_TASK_TARGET_HPP
#define RW_TASK_TARGET_HPP

/**
   @file Target.hpp
*/

#include "Link.hpp"
#include "Entity.hpp"
#include "ToolLocation.hpp"

#include <rw/math/Q.hpp>
#include <boost/variant.hpp>

namespace rw { namespace task {
	class Link;

	/** @addtogroup task */
    /*@{*/

    /**
       @brief Target represents a location for a device.

       The device location can for example be specified by transform for the
       tool of the device or by a configuration of joint values.
    */
	class Target : public Entity
	{
	public:
        //! Variant type for the different forms of device locations.
        typedef boost::variant<rw::math::Q, ToolLocation> Location;

        /**
           Constructor
        */
		Target(const Entity& entity, const Location& location);

        /**
           The location to which to move the device.
        */
        Location& getLocation() { return _value; }

        /**
           The location to which to move the device.
        */
		const Location& getLocation() const { return _value; }

	private:
		Location _value;
	};

    /**@}*/
}} // end namespaces

#endif // end include guard
