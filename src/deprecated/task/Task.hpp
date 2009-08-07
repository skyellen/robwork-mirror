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


#ifndef RW_TASK_TASK_HPP
#define RW_TASK_TASK_HPP

/**
 * @file Task.hpp
 */

#include "Entity.hpp"
#include "Trajectory.hpp"
#include "AttachFrame.hpp"

#include <rw/models/WorkCell.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/common/Ptr.hpp>

#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <ostream>

namespace rw { namespace task {

	/** @addtogroup task */
    /*@{*/

    /**
       @brief Task descriptions for workcells.

       A task contains a sequence of actions to carry out.
     */
	class Task : public Entity
	{
	public:
        //! Variant type for task actions.
        typedef boost::variant<AttachFrame, Trajectory> Action;

        //! Value type.
        typedef Action value_type;

		//! Iterator for the sequence of task actions.
		typedef std::vector<value_type>::iterator iterator;

		//! Const iterator for the sequence of task actions.
		typedef std::vector<value_type>::const_iterator const_iterator;

		/**
           @brief Constructor

           The workcell must be non-null.
        */
		Task(
            const Entity& entity,
            models::WorkCellPtr workcell,
            const std::vector<Action>& actions);

        /**
           The sequence of actions to carry out.
        */
        std::pair<iterator, iterator> getActions();

        /**
           The sequence of actions to carry out.
        */
        std::pair<const_iterator, const_iterator> getActions() const;

        /**
           @brief The workcell.
         */
        models::WorkCell& getWorkCell() const { return *_workcell.get(); }

	private:
        models::WorkCellPtr _workcell;
		std::vector<Action> _actions;
	};

    /**
       @brief Streaming operator.
    */
    std::ostream& operator<<(std::ostream& out, const Task& task);

}} // end namespaces

#endif
