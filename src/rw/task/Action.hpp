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

#ifndef RW_TASK_Action_HPP 
#define RW_TASK_Action_HPP 

/**
 * @file Action.hpp
 */

#include "TaskProperty.hpp"

#include <iostream>
#include <string.h>

namespace rw { namespace task {


	/** @addtogroup task */
    /*@{*/

    /**
     * @brief Data structure for task action specifications.
     *
	 * TODO: Longer description
     */


	class Action
	{	
	public:
		Action(std::string name);
		~Action();

		TaskProperty &Properties();

	private:
		std::string _name;

		TaskProperty _properties;


	};


}// end task namespace
}// end rw namespace

#endif