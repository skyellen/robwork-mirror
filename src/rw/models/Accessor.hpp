#ifndef ACCESSOR_HPP_
#define ACCESSOR_HPP_

#include <rw/kinematics/FrameProperty.hpp>
#include <rw/kinematics/FrameType.hpp>
#include <rw/proximity/CollisionSetup.hpp>

namespace rw { namespace models {
	/** @addtogroup models */
	/* @{ */

	/**
	 * @brief a set of accessor functions for accessing frame properties.
	 */
	class Accessor
	{
	public:
	    /** @brief Accessor for the setup for collision checking.
	     *
	     * This is used by CollisionDetector.
	     */
		static const kinematics::FrameProperty<proximity::CollisionSetup>& CollisionSetup();

	    /** @brief Accessor for the frame subtype identifier.
	     *
	     * We hope to remove this accessor at some point, and I don't think it is
	     * being used at all currently.
	     */
		static const kinematics::FrameProperty<kinematics::FrameType>& FrameType();

	    /** @brief Accessor for the GeoScale value.
	     *
	     * This should be used (if we want to support scaling at all) by the
	     * collision strategies, but this isn't done currently. It is used by
	     * drawables, however.
	     */
		static const kinematics::FrameProperty<double>& GeoScale();

	    /** @brief Accessor for the ActiveJoint flag.
	     *
	     * It is not yet clear at all how passive and active joints are handled, but
	     * currently the SerialDevice class assumes that joints it should care about
	     * have the ActiveJoint property. Therefore this property accessor is
	     * provided.
	     */
		static const kinematics::FrameProperty<bool>& ActiveJoint();

		 /**
		  * @brief Accessor for the DrawableID property
		  */
		static const kinematics::FrameProperty<std::string>& DrawableID();

		/**
		 * @brief Accessor for the CollisionModelID
		 */
		static const kinematics::FrameProperty<std::string>& CollisionModelID();

		 /**
            @brief Accessor for the DrawableHighlight property

            If the DrawableHighlight property exists, the drawable should
            initially be drawn highlighted.
         */
		static const kinematics::FrameProperty<bool>& DrawableHighlight();

		 /**
            @brief Accessor for the DrawableWireMode property

            If the DrawableHighlight property exists, the drawable should
            initially be drawn in wire mode.
         */
		static const kinematics::FrameProperty<bool>& DrawableWireMode();
	};

	/* @} */
}} // end namespaces

#endif // end include guard
