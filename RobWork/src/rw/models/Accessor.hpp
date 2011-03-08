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


#ifndef RW_MODELS_ACCESSOR_HPP
#define RW_MODELS_ACCESSOR_HPP

/**
   @file Accessor.hpp
*/


#include <rw/kinematics/FrameProperty.hpp>
#include <rw/kinematics/FrameType.hpp>
#include <rw/proximity/CollisionSetup.hpp>


#include "DrawableModelInfo.hpp"
#include "CollisionModelInfo.hpp"
#include "DHParameterSet.hpp"

namespace rw { namespace models {
	/** @addtogroup models */
	/* @{ */

    /* Some unfinished example code here:

	   @code
	   String
	   Frame *f;
	   if(
	   @endcode
	   Given a Frame *f, check if this frame has a CollisionModelInfo and
	   if it has get it.
    */

	/**
       @brief Accessors for frame properties.
	 */
	class Accessor
	{
	public:
		/**
           @brief Accessor for the setup for collision checking.

           This is used by CollisionDetector.
	     */
		//static const kinematics::FrameProperty<proximity::CollisionSetup>& collisionSetup();

	    /**
           @brief Accessor for the frame subtype identifier.

           We hope to remove this accessor at some point, and I don't think it
           is being used at all currently.
	     */
		//static const kinematics::FrameProperty<kinematics::FrameType>& frameType();

	    /**
         * @brief Accessor for the ActiveJoint flag.
	     */
		//static const kinematics::FrameProperty<bool>& activeJoint();


		/**
         * @brief Accessor for the DependentJoint flag.
         */
        //static const kinematics::FrameProperty<bool>& dependentJoint();


		/**
           @brief Accessor for the DrawableModelInfo property
        */
		//static const kinematics::FrameProperty<std::vector<DrawableModelInfo> >& drawableModelInfo();

		/**
           Accessor for the CollisionModelInfo property
        */
		//static const kinematics::FrameProperty<std::vector<CollisionModelInfo> >& collisionModelInfo();

		/**
           Accessor for Denavit-Hartenberg parameters.
        */
		//static const kinematics::FrameProperty<DHParameterSet>& dhSet();
	};

	/* @} */
}} // end namespaces

#endif // end include guard
