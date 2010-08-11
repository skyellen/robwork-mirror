/*
 * TreeDistanceCalc.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef TREEDISTANCECALC_HPP_
#define TREEDISTANCECALC_HPP_

#include "OBB.hpp"
#include "OBBTree.hpp"
#include "BVCollider.hpp"

/**
 * @brief this class encapsulates the methods for iterating through
 * two hierachical BV trees and finding the smallest distance between
 * any triangle in the trees.
 */
class TreeDistanceCalc {
private:
    TreeDistanceCalc(){};

public:
    virtual ~TreeDistanceCalc(){};

    /**
     *
     * @param fTA [in] transform from reference frame \b f to tree \b treeA root.
     * @param treeA [in]
     * @param fTB [in] transform from reference frame \b f to tree \b treeB root.
     * @param treeB [in]
     */
    virtual double distance(
        const rw::math::Transform3D<>& fTA, const CollisionModel& A,
        const rw::math::Transform3D<>& fTB, const CollisionModel& B) = 0;

    /**
     * @brief returns the amount of heap memmory used by the tree distance calculator.
     * @return nr of bytes used
     */
    virtual int getMemUsage() = 0;


    virtual int getNrOfTestedBVs(){ return -1;};
    virtual int getNrOfCollidingBVs(){ return -1;};

    virtual int getNrOfTestedPrimitives(){ return -1;};
    virtual int getNrOfCollidingPrimitives(){ return -1;};

    /**
     * @brief creates a tree collider that performes depth first search
     * of bv collisions between two hierarchical trees. The search is
     * balanced in the way that it switches between descending in the
     * trees.
     */
    static TreeDistanceCalc* makeBalancedDFSDistanceCalc(BVDistanceCalc* bvdistcalc);

    //TreeDistanceCalc* makeBalancedBFSCollider();

    /**
     * @brief creates a tree collider that performes depth first search
     * of bv collisions between two hierarchical trees. The search is
     * performed using a synchronous descent strategy which means that the
     * traversal descents both trees at the same time.
     */
    //static TreeDistanceCalc* makeSyncDFSCollider(BVCollider* bvcollider);

    /**
     *
     * @param weight
     * @return
     */
    //TreeDistanceCalc* makeWeightedDFSCollider(const BVWeight& weight);

    //TreeDistanceCalc* makeWeightedBFSCollider(const BVWeight& weight);

};


#endif /* TREECOLLIDER_HPP_ */
