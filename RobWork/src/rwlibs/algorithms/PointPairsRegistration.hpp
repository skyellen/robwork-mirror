/********************************************************************************
* Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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


#ifndef RWLIBS_ALGORITHMS_POINTPAIRSREGISTRATION_HPP
#define RWLIBS_ALGORITHMS_POINTPAIRSREGISTRATION_HPP


#include <rw/math/Transform3D.hpp>
#include <rw/common/PropertyMap.hpp>

namespace rwlibs {
	namespace algorithms {

		/**
		* Computes the transform minimizing: \f$sum_i(|T*p_i-p'_i|^2)\f$, where \f$p_i\f$ and \f$p'_i\f$ is a pair of points corresponding
		* to the same point in real world.
		*/
		class PointPairsRegistration
		{
		public:

			typedef std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > PointPair;

			/**
			 * @brief Perform registration of point pairs using SVD based method.
			 *
			 * At least three points are required for a registration
			 *
			 * The implemented method is described here: http://nghiaho.com/?page_id=671 . The implementation includes a step to take 
			 * into account if the rotation contains a relection rather than a pure rotation.
			 *
			 * @param pointPairs [in] Point pairs p_i and p'_i to register.
			 * @return Transform minimising the squared distance between point pairs
			 */
			static rw::math::Transform3D<> pointPairRegistrationSVD(const std::vector<PointPair>& pointPairs);

			/**
			* @brief Perform registration of point pairs using Quaternion based method.
			*
			* At least three points are required for a registration
			*
			* The method implemented is Besl, P. J. Et al.: A Method for Registration of 3-D Shapes. PAMI, 14(2):239-256, 1992.
			*			
			* @param pointPairs [in] Point pairs p_i and p'_i to register.
			* @return Transform minimising the squared distance between point pairs
			*/
			static rw::math::Transform3D<> pointPairRegistrationQuaternion(const std::vector<PointPair>& pointPairs);

		};

	} //end namespace
}//end namespace

#endif //#ifndef RWLIBS_ALGORITHMS_POINTPAIRSREGISTRATION_HPP
