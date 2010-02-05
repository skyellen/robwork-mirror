/*
 * ContactCluster.hpp
 *
 *  Created on: 23-10-2008
 *      Author: jimali
 */

#ifndef CONTACTCLUSTER_HPP_
#define CONTACTCLUSTER_HPP_

#include "ContactPoint.hpp"

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

};


#endif /* CONTACTCLUSTER_HPP_ */
