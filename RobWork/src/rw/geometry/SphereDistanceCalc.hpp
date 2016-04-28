/*
 * BVCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_SPHEREDISTANCECALC_HPP_
#define RW_PROXIMITY_SPHEREDISTANCECALC_HPP_

#include <sandbox/geometry/OBB.hpp>
#include "BVDistanceCalc.hpp"
#include <rw/math/Vector3D.hpp>
#include "BSphere.hpp"
namespace rw {
namespace proximity {

	/**
	 * @brief class for testing if two Oriented Bounding Boxes are overlapping
	 */

	template<class T=double>
	class SphereDistanceCalc : public BVDistanceCalc<SphereDistanceCalc<T>, rw::geometry::BSphere<T> > {
	public:
		typedef T value_type;

		//! @brief constructor
		SphereDistanceCalc(){};

		//! @brief destructor
		virtual ~SphereDistanceCalc(){};

		/**
		 * @brief Calculates the distance between two bounding spheres.
		 */
		inline double distance(const rw::geometry::BSphere<T>& a,
								 const rw::geometry::BSphere<T>& b,
								 const rw::math::Vector3D<T>& aTb)
		{
		    return aTb.norm2()-(a.getRadius()+b.getRadius());
		}

        /**
         * @brief calculates the squared distance between two bounding spheres.
         */
        inline double distanceSqr(const rw::geometry::BSphere<T>& a,
                                 const rw::geometry::BSphere<T>& b,
                                 const rw::math::Vector3D<T>& aTb)
        {
            return MetricUtil::norm2Sqr(aTb)-(a.getRadiusSqr()+b.getRadiusSqr());
        }

	};

}
}

#endif
