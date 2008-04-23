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
 * for detailed information about these packages.
 *********************************************************************/

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
