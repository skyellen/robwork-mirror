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

#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <ostream>

namespace rw { namespace task {

	/** @addtogroup task */
    /*@{*/

    /**
       @brief Task descriptions for workcells.
     */
	class Task : public Entity
	{
	public:
        //! Variant type for Task actions.
        typedef boost::variant<AttachFrame, Trajectory> Action;

        //! Value type.
        typedef Action value_type;

        //! Pointer type.
        typedef value_type* pointer;

        //! Reference type.
        typedef value_type& reference;

		//! Iterator for the sequence of task actions.
		typedef std::vector<value_type>::iterator iterator;

		//! Const iterator for the sequence of task actions.
		typedef std::vector<value_type>::const_iterator const_iterator;

        //! Iterator category.
        typedef iterator::iterator_category iterator_category;

        //! Difference type.
        typedef iterator::difference_type difference_type;

		/**
           @brief Constructor

           Ownership of the workcell is not taken.
        */
		Task(
            const Entity& entity,
            models::WorkCell* workcell);

        /**
           @brief Constructor

           Ownership of the workcell is taken.
         */
		Task(
            const Entity& entity,
            std::auto_ptr<models::WorkCell> workcell);

		void addAction(const Action& action);

        std::pair<iterator, iterator> getValues()
        {
            return std::make_pair(_actions.begin(), _actions.end());
        }

        std::pair<const_iterator, const_iterator> getValues() const
        {
            return std::make_pair(_actions.begin(), _actions.end());
        }

        /**
           @brief The workcell.
         */
        models::WorkCell& getWorkCell() const { return *_workcell; }

	private:
        models::WorkCell* _workcell;
        boost::shared_ptr<models::WorkCell> _own_workcell;
		std::vector<Action> _actions;
	};

    /**
       @brief Streaming operator.
    */
    std::ostream& operator<<(std::ostream& out, const Task& task);

}} // end namespaces

#endif
