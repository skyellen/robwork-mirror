/*
 * TreeCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_BVTREECOLLIDER_HPP_
#define RW_PROXIMITY_BVTREECOLLIDER_HPP_

#include <sandbox/geometry/OBB.hpp>
#include "BVTree.hpp"
#include "BinaryBVTree.hpp"
#include "BVCollider.hpp"
#include <sandbox/geometry/OBB.hpp>
#include "OBBCollider.hpp"

namespace rw {
namespace proximity {

	/**
	 * @brief this class encapsulates the methods for iterating through
	 * two hierachical OBV trees while testing if the BV's are disjoint.
	 */
	template<class BVTREE>
	class BVTreeCollider {
	private:

	public:
		virtual ~BVTreeCollider(){};

		/**
		 *
		 * @param fTA [in] transform from reference frame \b f to tree \b treeA root.
		 * @param treeA [in]
		 * @param fTB [in] transform from reference frame \b f to tree \b treeB root.
		 * @param treeB [in]
		 */
		virtual bool collides(
			const rw::math::Transform3D<>& fTA, const BVTree& treeA,
			const rw::math::Transform3D<>& fTB, const BVTree& treeB) = 0;

		/**
		 * @brief returns the amount of heap memmory used by the tree collider.
		 * @return nr of bytes used
		 */
		virtual int getMemUsage() = 0;

		virtual int getNrOfTestedBVs(){ return -1;};
		virtual int getNrOfCollidingBVs(){ return -1;};

		virtual int getNrOfTestedPrimitives(){ return -1;};
		virtual int getNrOfCollidingPrimitives(){ return -1;};


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


	class BVTreeColliderFactory {
	public:

		template<class DERIVED, class BVTREE>
		struct BVDescentStrategy {
			typedef typename BVTREE::BVNode BVNODE;

			bool descentIntoA(const BVNODE& bvA, const BVNODE& bvB){
				return static_cast<DERIVED*>(this)->descentIntoA(bvA,bvB);
			}

		};

		template<class BVTREE>
		struct BalancedDescentStrategy: public BVDescentStrategy< BalancedDescentStrategy<BVTREE>, BVTREE>{
			typedef typename BVTREE::BVNode BVNODE;

			BalancedDescentStrategy():previousRes(true){};
			bool descentIntoA(const BVNODE& bvA, const BVNODE& bvB){
				previousRes = !previousRes;
				return previousRes;
			}

			bool previousRes;
		};

		/**
		 * @brief creates a tree collider that performes depth first search
		 * of bv collisions between two hierarchical trees. The search is
		 * balanced in the way that it equally switches between descending in the
		 * trees.
		 */
		template<class BVTREE>
		static BVTreeCollider<BVTREE>* makeBalancedDFSColliderOBB(){
			OBBCollider<typename BVTREE::value_type>* bvcollider = new OBBCollider<typename BVTREE::value_type>();
			BalancedDescentStrategy<typename BVTREE::BVType>* dstrategy = new BalancedDescentStrategy<BVTREE>();
			return makeDFSCollider<OBBCollider, BalancedDescentStrategy, BVTREE>(bvcollider, dstrategy);
		}

		/**
		 * @brief creates a depth first search tree collider
		 * @param bvcollider
		 * @param dstrat
		 * @return
		 */
		template<class COLLIDER, class DESCENT_STRATEGY, class BVTREE>
		static BVTreeCollider<BVTREE>* makeDFSCollider(COLLIDER* bvcollider, DESCENT_STRATEGY* dstrat);
	};

}
}


#endif /* TREECOLLIDER_HPP_ */
