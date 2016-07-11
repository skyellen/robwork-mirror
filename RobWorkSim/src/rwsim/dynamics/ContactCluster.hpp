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

#ifndef RWSIM_DYNAMICS_CONTACTCLUSTER_HPP_
#define RWSIM_DYNAMICS_CONTACTCLUSTER_HPP_

namespace rwsim {
namespace dynamics {
	class ContactPoint;

	//! @addtogroup @{

	/**
	 * @brief calculates contact clusters
	 */
	class ContactCluster {

	public:

		/**
		 * @brief
		 * @param points
		 * @param clusters
		 */
		//static void calcClusters(const std::vector<ContactPoint>& points,
		//                         std::vector<ContactCluster>& clusters);

		static int thresClustering(
						ContactPoint src[], int srcCnt,
						int *cIdxSrc, int *cIdxDst,
						ContactPoint dst[],
						double maxDist);


		/**
		 * @brief this method cluster together contacts that have equal/close normals.
		 * @param src
		 * @param srcCnt
		 * @param cIdxSrc
		 * @param cIdxDst
		 * @param dst
		 * @param maxDist
		 * @return
		 */
		static int normalThresClustering(ContactPoint src[],
		                                 int srcCnt,
		                                 int *cIdxSrc,
		                                 int *cIdxDst,
		                                 ContactPoint dst[],
		                                 double maxDist);



	};
	//! @}
}
}

#endif /* CONTACTCLUSTER_HPP_ */
