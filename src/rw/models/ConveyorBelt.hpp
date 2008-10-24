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
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_MODELS_CONVEYORBELT_HPP
#define RW_MODELS_CONVEYORBELT_HPP

#include "ConveyorSegment.hpp"
#include "ConveyorItem.hpp"

#include <rw/trajectory/Trajectory.hpp>

namespace rw { namespace models {

	/** @addtogroup models */
	/* @{ */

	/**
	 * @brief A simple conveyor belt to be used with a Conveyor
	 *
	 * The ConveyerBelt provides a simple ConveyorSegment with 1 entry and 1 exit point. Associated
	 * with a belt is a trajectory, which is used to calculate the position of items.
	 */
	class ConveyorBelt: public ConveyorSegment
	{
	public:
		/**
		 * @brief Constructor
		 */
		ConveyorBelt(const rw::trajectory::Trajectory<rw::math::Q>& trajectory,
		             rw::kinematics::Frame* baseFrame,
		             ConveyorSegment* next,
		             ConveyorSegment* previous);

		/**
		 * @brief Destructor
		 */
		virtual ~ConveyorBelt();

		/**
		 * @copydoc ConveyorSegment::addItem
		 */
	    virtual void addItem(ConveyorItem* item, FramePosition position, rw::kinematics::State& state);

		/**
		 * @copydoc ConveyorSegment::move
		 */
	    virtual void move(ConveyorItem* item, double d, rw::kinematics::State& state);

		/**
		 * @copydoc ConveyorSegment::length
		 */
	    virtual double length() const;

		/**
		 * @copydoc ConveyorSegment::getBaseFrame
		 */
	    virtual rw::kinematics::Frame* getBaseFrame();

		/**
		 * @brief Sets the previous segment
		 * @param segment [in] the segment to use as the previous
		 */
		virtual void setPreviousSegment(ConveyorSegment* segment);

		/**
		 * @brief Sets the next segment
		 * @param segment [in] the segment to use as the next
		 */
		virtual void setNextSegment(ConveyorSegment* segment);


	private:
	    /**
	     * Specifies
	     */
	    virtual void moveTo(ConveyorItem* item, double q, rw::kinematics::State& state);

	    rw::trajectory::Trajectory<rw::math::Q> _trajectory;
	    rw::kinematics::Frame* _baseFrame;
        ConveyorSegment* _next;
	    ConveyorSegment* _previous;

	    std::map<rw::kinematics::Frame*, double> _frame2q;
	};
	/* @} */
}} // end namespaces

#endif //#ifndef RW_MODELS_CONVEYORBELT_HPP
