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
#ifndef RW_TASK_LINK_HPP 
#define RW_TASK_LINK_HPP 

/**
 * @file Link.hpp
 */

#include "Target.hpp"


namespace rw { namespace task {
	class Target;


	/** @addtogroup task */
    /*@{*/

    /**
     * @brief Data structure for Link specifications in task trajectories.
     *
	 * TODO: Longer description
     */


	class Link
	{	
		friend class TaskTrajectory;
	public:
		enum SpeedType {Angular, Positional};
		
		Link(double tool_speed, SpeedType speed_type);

		TaskProperty &Properties() { return _properties; };
		
		Target *Next() { return _next; }
		Target *Prev() { return _prev; }


	private:
		void setNext(Target *next) { _next = next; }
		void setPrev(Target *prev) { _prev = prev; } 

		double _tool_speed;
		SpeedType _speed_type;

		TaskProperty _properties;

		Target *_prev, *_next;


	};


}// end task namespace
}// end rw namespace

#endif

