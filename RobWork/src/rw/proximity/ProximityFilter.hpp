/*
 * ProximityData.hpp
 *
 *  Created on: 23/04/2010
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_PROXIMITYFILTER_HPP
#define RW_PROXIMITY_PROXIMITYFILTER_HPP

#include <rw/kinematics/Frame.hpp>


namespace rw {
namespace proximity {

	/**
	 * @brief this class is used for fetching frame pairs using some proximity filtering strategy.
	 *
	 * The proximity filter is statefull and in the simplest case its an iterator over a set of frame pairs.
	 *
	 * The filter implementations should support early existing, to reduce computations.
	 */
	class ProximityFilter {
	public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<ProximityFilter> Ptr;

		/**
		 * @brief returns the next possibly colliding framepair.
		 * @return a frame pair
		 */
		virtual void pop() = 0;

		/**
		 * @brief returns the current front and pops it afterwards
		 * @return the current front element
		 */
		virtual rw::kinematics::FramePair frontAndPop() = 0;

		/**
		 * @brief if there are any more possibly colliding framepairs since last
		 * call to update then this will return true, else false will be returned.
		 */
		virtual rw::kinematics::FramePair front() = 0;


		/**
		 * @brief if there are any more possibly colliding framepairs since last
		 * call to update then this will return true, else false will be returned.
		 */
		virtual bool isEmpty() = 0;

	};

}
}

#endif /* PROXIMITYDATA_HPP */
