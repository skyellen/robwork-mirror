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

		/*
		 * @brief
		 * @param points
		 * @param clusters
		 */
		//static void calcClusters(const std::vector<ContactPoint>& points,
		//                         std::vector<ContactCluster>& clusters);

		/**
		 * @brief Cluster together contacts using a simple distance threshold.
		 *
		 * The method takes the point with the deepest penetration first, then takes the points
		 * within the distance threshold and creates a cluster. Then the process is repeated
		 * for the remaining contacts until all contacts have been treated.
		 *
		 * The resulting contacts in \b dst is one contact from each cluster, and it is that one
		 * contact within each cluster with the largest penetration.
		 *
		 * The first elements in \b cIdxSrc gives the index of the deepest point in each cluster in
		 * the original \b src list.
		 *
		 * @param src [in] the list of input contact points.
		 * @param srcCnt [in] the number of contacts in \b src.
		 * @param cIdxSrc [out] a vector of indices where \b srcCnt elements must be reserved.
		 * @param cIdxDst [out] a vector of indices where \b srcCnt elements must be reserved.
		 * @param dst [out] the result of clustering.
		 * @param maxDist [in] the norm-2 distance threshold for the contact positions.
		 * @return the number of clusters (the number of contacts in \b dst).
		 */
		static int thresClustering(
						ContactPoint src[], int srcCnt,
						int *cIdxSrc, int *cIdxDst,
						ContactPoint dst[],
						double maxDist);


		/**
		 * @brief Cluster together contacts that have equal/close normals.
		 *
		 * The method takes the point with the deepest penetration first, then takes the contacts
		 * with normals within the threshold and creates a cluster. Then the process is repeated
		 * for the remaining contacts until all contacts have been treated.
		 *
		 * The resulting contacts in \b dst is one contact from each cluster, and it is that one
		 * contact within each cluster with the largest penetration.
		 *
		 * The first elements in \b cIdxSrc gives the index of the deepest point in each cluster in
		 * the original \b src list.
		 *
		 * @param src [in] the list of input contact points.
		 * @param srcCnt [in] the number of contacts in \b src.
		 * @param cIdxSrc [out] a vector of indices where \b srcCnt elements must be reserved.
		 * @param cIdxDst [out] a vector of indices where \b srcCnt elements must be reserved.
		 * @param dst [out] the result of clustering.
		 * @param maxDist [in] the norm-2 distance threshold for the contact normals.
		 * @return the number of clusters (the number of contacts in \b dst).
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
