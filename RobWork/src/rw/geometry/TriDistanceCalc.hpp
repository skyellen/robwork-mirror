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

#ifndef RW_PROXIMITY_TRIDISTANCECALC_HPP_
#define RW_PROXIMITY_TRIDISTANCECALC_HPP_

#include <rw/geometry/Triangle.hpp>

#include <PQP/PQP.h>
#include <PQP/TriDist.h>

#include "BVDistanceCalc.hpp"

namespace rw {
namespace geometry {

	/**
	 * @brief class for testing if two Oriented Bounding Boxes are overlapping
	 */

	template<class T=double>
	class TriDistanceCalc : public BVDistanceCalc<TriDistanceCalc<T>, rw::geometry::Triangle<T> > {
	public:
		typedef T value_type;

		//! @brief constructor
		TriDistanceCalc(){};

		//! @brief destructor
		virtual ~TriDistanceCalc(){};


        T distance(const rw::geometry::Triangle<T>& a,
                   const rw::geometry::Triangle<T>& b);

		/**
		 * @brief test if obbA intersects obbB. The aTb transform describe
		 * obbB relative to obbA's coordinate frame
		 */
		T distance(const rw::geometry::Triangle<T>& a,
		           const rw::geometry::Triangle<T>& b,
				   const rw::math::Transform3D<T>& aTb);
	private:

		void toPQPTri(const rw::geometry::Triangle<T>& tri1, PQP::PQP_REAL dst[3][3]){
		    dst[0][0] = (PQP::PQP_REAL)tri1[0][0];
		    dst[0][1] = (PQP::PQP_REAL)tri1[0][1];
		    dst[0][2] = (PQP::PQP_REAL)tri1[0][2];

		    dst[1][0] = (PQP::PQP_REAL)tri1[1][0];
		    dst[1][1] = (PQP::PQP_REAL)tri1[1][1];
		    dst[1][2] = (PQP::PQP_REAL)tri1[1][2];

		    dst[2][0] = (PQP::PQP_REAL)tri1[2][0];
		    dst[2][1] = (PQP::PQP_REAL)tri1[2][1];
		    dst[2][2] = (PQP::PQP_REAL)tri1[2][2];
		}


	};

	template<class T>
	T TriDistanceCalc<T>::distance(const rw::geometry::Triangle<T>& a,
	                               const rw::geometry::Triangle<T>& b,
	                               const rw::math::Transform3D<T>& aTb)
    {
	    rw::geometry::Triangle<T> bInaT = b.transform( aTb );
	    return distance(a,bInaT);
    }

    template<class T>
    T TriDistanceCalc<T>::distance(const rw::geometry::Triangle<T>& a,
                                   const rw::geometry::Triangle<T>& b)
    {
        T tri1[3][3], tri2[3][3], P1[3], P2[3];
        toPQPTri(a, tri1);
        toPQPTri(b, tri2);
        return PQP::TriDist(P1,P2,tri1,tri2);
    }

}
}

#endif
