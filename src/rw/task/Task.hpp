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

#ifndef RW_TASK_TASK_HPP
#define RW_TASK_TASK_HPP

/**
 * @file Task.hpp
 */

#include "Trajectory.hpp"
#include "Action.hpp"


#include <boost/variant.hpp>
#include <vector>


namespace rw { namespace task {


	/** @addtogroup task */
    /*@{*/

    /**
     * @brief Data structure for task specification.
     *
	 * TODO: Longer description
     */


	class Task
	{	
	public:

        //! Variant type for Task elements
        typedef boost::variant<Action, Trajectory > TaskElement;

		//!Iterator for the vector of taskelements
		typedef std::vector<TaskElement>::iterator iterator;



		/**
        * @brief Constructs a Task object
        */
		Task() { }


        /**
           @brief Destructor.
        */
		~Task() { }

		void addTaskElement(TaskElement task_element);



        /**
         * @brief Get an iterator to the first task element
         *
         * @param to [out] Iterator pointing to the first task element.
         */
		iterator begin() { return _task_elements.begin(); }


        /**
         * @brief Get an iterator to the last task element
         *
         * @param to [out] Iterator pointing past the last task element.
         */
		iterator end() { return _task_elements.end(); }
			

	private:

		std::vector<TaskElement> _task_elements;

	};


}// end task namespace
}// end rw namespace

#endif
