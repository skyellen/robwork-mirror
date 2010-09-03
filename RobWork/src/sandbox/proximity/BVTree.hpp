/*
 * BVTree.hpp
 *
 *  Created on: 05-02-2009
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_BVTREE_HPP_
#define RW_PROXIMITY_BVTREE_HPP_

#include <sandbox/geometry/OBB.hpp>


namespace rw {
namespace proximity {

	/**
	 * @brief this implementation defines a BVTree structure that use
	 * an indexed based storage representation. The implementation does not include functionality
	 * for building a BVTree. It is a data structure for accessing nodes in a bounding volume tree.
	 *
	 */
	class BVTree {
	public:
		//typedef typename DERIVED::nodeiterator Node;
		//typedef typename DERIVED::BVType BVType;

		//typedef NodeIterator iterator;

		template<class DERIVED_NODE, class BVTYPE>
		struct Node {
			typedef typename BVTYPE::value_type value_type;
			typedef BVTYPE BVType;
			//! @brief constructor
			inline const BVType& getBV() const { return static_cast<const DERIVED_NODE*>(this)->bv(); };
			inline bool isLeaf() const { return static_cast<const DERIVED_NODE*>(this)->leaf(); };
			inline Node left() const { return static_cast<const DERIVED_NODE*>(this)->left(); };
			inline Node right() const { return static_cast<const DERIVED_NODE*>(this)->right(); };
			inline unsigned char depth() const { return static_cast<const DERIVED_NODE*>(this)->depth(); };
		};


		template <class DERIVED>
		typename DERIVED::nodeiterator getRoot(){ return static_cast<DERIVED*>(this)->getRoot(); }

		virtual int getMaxTrisPerLeaf() const = 0;
	private:

	};

}
}

#endif /* BVTREE_HPP_ */
