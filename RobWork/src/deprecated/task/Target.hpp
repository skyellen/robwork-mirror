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
