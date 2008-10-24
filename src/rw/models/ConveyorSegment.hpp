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

#ifndef RW_MODELS_CONVEYORSEGMENT_HPP
#define RW_MODELS_CONVEYORSEGMENT_HPP

#include <rw/kinematics/Frame.hpp>
#include <rw/models/ConveyorItem.hpp>

namespace rw {
namespace models {
	/** @addtogroup models */
	/* @{ */

	/**
	 * @brief Segment of a Conveyor system
	 *
	 * A Conveyor consist of one or more ConveyorSegments. The ConveyorSegment provides an interface
	 * which all segments most fulfil.
	 */
	class ConveyorSegment
	{
	public:
		/**
		 * @brief default constructor
		 */
		ConveyorSegment();

		/**
		 * @brief default destructor
		 */
		virtual ~ConveyorSegment();

		/**
		 * @brief Returns the base frame of the segment
		 *
		 * @return The base frame
		 */
		virtual rw::kinematics::Frame* getBaseFrame() = 0;


		/**
		 * @brief Enumeration used to specify start and end of the conveyor segment
		 */
		enum FramePosition { START = 0, END };

		/**
		 * @brief Adds item to the ConveyorSegment
		 *
		 * When added the daf-parent of the item becomes the ConveyorSegment's baseframe and
		 * the pose of the item gets controlled by the segment.
		 *
		 * @param item [in] The item to add
		 * @param position [in] Whether to add it in the start of end of the conveyor
		 * @param state [inout] State of the workcell
		 */
		virtual void addItem(
            ConveyorItem* item,
            FramePosition position,
            rw::kinematics::State& state) = 0;

		/**
		 * @brief Moves a segment by an amount d
		 *
		 * @param item [in] The item to move
		 * @param d [in] The amount to move
		 * @param state [in] State of the workcell
		 */
		virtual void move(ConveyorItem* item, double d, rw::kinematics::State& state) = 0;

		/**
		 * @brief Length of the segment
		 * @return the length of the segment
		 */
		virtual double length() const = 0;


	};
	/* @} */
}
}

#endif /*RW_MODELS_CONVEYORSEGMENT_HPP*/
