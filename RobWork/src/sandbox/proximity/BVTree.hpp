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

    template<class DERIVED_NODE, class BVTYPE>
    struct BVTreeNode {
        typedef typename BVTYPE::value_type value_type;
        typedef BVTYPE BVType;
        //! @brief constructor
        DERIVED_NODE* downcast(){ return static_cast<const DERIVED_NODE*>(this); }
        const DERIVED_NODE* downcast() const { return static_cast<const DERIVED_NODE*>(this); }

        inline const BVType& getBV() const { return downcast()->bv(); };
        inline bool isLeaf() const { return downcast()->leaf(); };
        inline DERIVED_NODE left() const { return downcast()->left(); };
        inline DERIVED_NODE right() const { return downcast()->right(); };
        inline bool hasRight(){ return downcast()->hasRight(); };
        inline bool hasLeft(){ return downcast()->hasLeft(); };
        inline unsigned char depth() const { return downcast()->depth(); };
        inline std::vector<rw::geometry::Triangle<value_type> > getPrimitives() const {
            return downcast()->getPrimitives();
        };
    };

	/**
	 * @brief this implementation defines a BVTree structure that use
	 * an indexed based storage representation. The implementation does not include functionality
	 * for building a BVTree. It is a data structure for accessing nodes in a bounding volume tree.
	 *
	 */
    template<class NODEITERATOR>
	class BVTree{
	public:
		typedef typename NODEITERATOR::BVType BVType;
		typedef NODEITERATOR Node;

		virtual NODEITERATOR getRootIterator() const = 0;//{ return static_cast<DERIVED*>(this)->getRoot(); }
		virtual int getMaxTrisPerLeaf() const = 0;

		virtual NODEITERATOR createLeft( NODEITERATOR parent) = 0;
		virtual NODEITERATOR createRight( NODEITERATOR parent ) = 0;
		virtual NODEITERATOR createRoot() = 0;

		virtual void setBV(const BVType& bv, NODEITERATOR node) = 0;
        virtual void setNrOfPrims(int size, NODEITERATOR node) = 0;
        virtual void setPrimIdx(int primIdx, NODEITERATOR node) = 0;



        virtual void compile() = 0;
	private:

	};

}
}

#endif /* BVTREE_HPP_ */
