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
