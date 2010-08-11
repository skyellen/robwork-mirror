/*
 * TreeCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef TREECOLLIDER_HPP_
#define TREECOLLIDER_HPP_

#include "OBB.hpp"
#include "OBBTree.hpp"
#include "BVCollider.hpp"

/**
 * @brief this class encapsulates the methods for iterating through
 * two hierachical OBV trees while testing if the BV's are disjoint.
 */
class TreeCollider {
private:
    //TreeCollider(){};

public:
    virtual ~TreeCollider(){};

    /**
     *
     * @param fTA [in] transform from reference frame \b f to tree \b treeA root.
     * @param treeA [in]
     * @param fTB [in] transform from reference frame \b f to tree \b treeB root.
     * @param treeB [in]
     */
    virtual bool collides(
        const rw::math::Transform3D<>& fTA, const OBBTree<>& treeA,
        const rw::math::Transform3D<>& fTB, const OBBTree<>& treeB) = 0;

    /**
     * @brief returns the amount of heap memmory used by the tree collider.
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
    static TreeCollider* makeBalancedDFSCollider(BVCollider* bvcollider);

    //TreeCollider* makeBalancedBFSCollider();

    /**
     * @brief creates a tree collider that performes depth first search
     * of bv collisions between two hierarchical trees. The search is
     * performed using a synchronous descent strategy which means that the
     * traversal descents both trees at the same time.
     */
    //static TreeCollider* makeSyncDFSCollider(BVCollider* bvcollider);

    /**
     *
     * @param weight
     * @return
     */
    //TreeCollider* makeWeightedDFSCollider(const BVWeight& weight);

    //TreeCollider* makeWeightedBFSCollider(const BVWeight& weight);

};


#endif /* TREECOLLIDER_HPP_ */
