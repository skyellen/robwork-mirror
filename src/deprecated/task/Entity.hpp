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


#ifndef RW_TASK_ENTITY_HPP
#define RW_TASK_ENTITY_HPP

/**
   @file Entity.hpp
*/

#include <string>
#include <rw/common/PropertyMap.hpp>

namespace rw { namespace task {

	/** @addtogroup task */
    /*@{*/

    /**
       @brief Entity stores the name and property map for an element of a task
       description.

       The convention of the task data types is simply to subclass the Entity
       data type.

       Values of type Entity can and are supposed to be copied and assigned
       freely.
     */
	class Entity
	{
	public:
        /**
           @brief Constructor
        */
		Entity(
            const std::string& name,
            const common::PropertyMap& properties)
            :
            _name(name),
            _properties(properties)
        {}

        /**
           @brief The name of the task entity.
         */
		const std::string& getName() const { return _name; }

        /**
           @brief The property map of the task entity.
         */
		common::PropertyMap& getPropertyMap() { return _properties; }

        /**
           @brief The property map of the task entity.
        */
		const common::PropertyMap& getPropertyMap() const { return _properties; }

	private:
		std::string _name;
		common::PropertyMap _properties;
	};

    /**@}*/
}} // end namespaces

#endif // end include guard
