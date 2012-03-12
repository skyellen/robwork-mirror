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
#include <sandbox/geometry/TRIDeviller.hpp>
#include <boost/foreach.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>


namespace rw {
namespace proximity {

	/**
	 * @brief this class encapsulates the methods for iterating through
	 * two hierachical OBV trees while testing if the BV's are disjoint.
	 *
	 *
	 * @note The TreeCollider is statefull and should therefore NOT be used by
	 * multiple threads. The state is small so there is only little to no overhead
	 * cloning the TreeCollider.
	 */
    template<class BVTREE>
	class BVTreeCollider {
	private:

	public:
        //! @brief smart pointer for this class
        typedef rw::common::Ptr<BVTreeCollider<BVTREE> > Ptr;

        /**
         * @brief destructor
         */
		virtual ~BVTreeCollider(){};

		/**
		 * @brief tests if two BV trees are colliding.
		 * @param fTA [in] transform from reference frame \b f to tree \b treeA root.
		 * @param treeA [in]
		 * @param fTB [in] transform from reference frame \b f to tree \b treeB root.
		 * @param treeB [in]
		 */
		virtual bool collides(
			const rw::math::Transform3D<typename BVTREE::value_type>& fTA, const BVTREE& treeA,
			const rw::math::Transform3D<typename BVTREE::value_type>& fTB, const BVTREE& treeB,
			std::vector<std::pair<int,int> > *collidingPrimitives = NULL) = 0;

		/**
		 * @brief set the query type
		 */
		virtual void setQueryType(CollisionStrategy::QueryType type){ _queryType=type;};

        /**
         * @brief get the collision query type
         */
		virtual CollisionStrategy::QueryType getQueryType(){ return _queryType;};

		//! type of the primitive in collision callb ack function
		//typedef boost::function<void(int,int)> PrimitivesInCollisionCB;
		//virtual void setPrimitivesInCollisionCB(PrimitivesInCollisionCB cb){ _pCB = cb;}

		/**
		 * @brief returns the amount of heap memmory used by the tree collider.
		 * @return nr of bytes used
		 */
		virtual int getMemUsage() = 0;


		virtual int getNrOfTestedBVs(){ return -1;};
		virtual int getNrOfCollidingBVs(){ return -1;};

		virtual int getNrOfTestedPrimitives(){ return -1;};
		virtual int getNrOfCollidingPrimitives(){ return -1;};

	protected:
		CollisionStrategy::QueryType _queryType;
		//PrimitivesInCollisionCB _pCB;
	};


}
}


#endif /* TREECOLLIDER_HPP_ */
