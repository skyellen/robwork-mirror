#ifndef RW_MODELS_ACCESSOR_HPP
#define RW_MODELS_ACCESSOR_HPP

/**
   @file Accessor.hpp
*/

#include <rw/kinematics/FrameProperty.hpp>
#include <rw/kinematics/FrameType.hpp>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/invkin/PieperSolver.hpp>

#include "DrawableModelInfo.hpp"
#include "CollisionModelInfo.hpp"

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
		static const kinematics::FrameProperty<proximity::CollisionSetup>&
        collisionSetup();

	    /**
           @brief Accessor for the frame subtype identifier.

           We hope to remove this accessor at some point, and I don't think it
           is being used at all currently.
	     */
		static const kinematics::FrameProperty<kinematics::FrameType>&
        frameType();

	    /**
           @brief Accessor for the ActiveJoint flag.

           It is not yet clear at all how passive and active joints are handled,
           but currently the SerialDevice class assumes that joints it should
           care about have the ActiveJoint property. Therefore this property
           accessor is provided.
        */
		static const kinematics::FrameProperty<bool>&
        activeJoint();

        /**
           @brief Accessor for the DrawableModelInfo property
        */
		static const kinematics::FrameProperty<std::vector<DrawableModelInfo> >&
        drawableModelInfo();

		/**
           Accessor for the CollisionModelInfo property
        */
		static const kinematics::FrameProperty<std::vector<CollisionModelInfo> >&
        collisionModelInfo();

		/**
           Accessor for Denavit-Hartenberg parameters.
        */
		static const kinematics::FrameProperty<rw::invkin::DHSet>&
        dhSet();
	};

	/* @} */
}} // end namespaces

#endif // end include guard
