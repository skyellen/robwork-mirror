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
