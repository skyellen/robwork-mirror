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

        inline DERIVED_NODE* downcast(){ return static_cast<const DERIVED_NODE*>(this); }
        inline const DERIVED_NODE* downcast() const { return static_cast<const DERIVED_NODE*>(this); }

        inline int getId()const { return downcast()->getId(); };
        inline const BVType& getBV() const { return downcast()->bv(); };
        inline bool isLeaf() const { return downcast()->leaf(); };
        inline DERIVED_NODE left() const { return downcast()->left(); };
        inline DERIVED_NODE right() const { return downcast()->right(); };
        inline bool hasRight() const { return downcast()->hasRight(); };
        inline bool hasLeft() const { return downcast()->hasLeft(); };
        inline unsigned char depth() const { return downcast()->depth(); };

        inline size_t triangleIdx() const {return downcast()->triangleIdx();}
        inline size_t nrOfTriangles() const { return downcast()->nrOfTriangles();}

        //inline std::vector<rw::geometry::Triangle<value_type> > getPrimitives() const {
        //    return downcast()->getPrimitives();
        //};
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
		typedef typename BVType::value_type value_type;
		typedef NODEITERATOR Node;
		BVTree(rw::geometry::TriMesh::Ptr triangles):
		    _triangles(triangles)
		{}

		virtual NODEITERATOR getRootIterator() const = 0;//{ return static_cast<DERIVED*>(this)->getRoot(); }
		virtual int getMaxTrisPerLeaf() const = 0;

		virtual NODEITERATOR createLeft( NODEITERATOR parent) = 0;
		virtual NODEITERATOR createRight( NODEITERATOR parent ) = 0;
		virtual NODEITERATOR createRoot() = 0;

		virtual void setBV(const BVType& bv, NODEITERATOR node) = 0;
        virtual void setNrOfPrims(int size, NODEITERATOR node) = 0;
        virtual void setPrimIdx(int primIdx, NODEITERATOR node) = 0;

        virtual void compile() = 0;

        inline void getTriangle(const NODEITERATOR& leafnode,rw::geometry::Triangle<value_type>& dst, size_t triNr) const {
            RW_ASSERT(leafnode.nrOfTriangles()>0);
            size_t idx = leafnode.triangleIdx();
            //std::cout << "Idx: " << idx+triNr << "  " << idx<<"+"<<triNr<< "\n";
            _triangles->getTriangle(idx+triNr, dst);
            //std::cout << dst[0] << "\n";
        }
        /*inline rw::geometry::Triangle<value_type>* getTriangles(NODEITERATOR& leafnode){
            RW_ASSERT(leafnode.nrOfTriangles()>0);
            leafnode.triangleIdx();
            return downcast()->getTriangles();
        }*/

        inline size_t getNrTriangles(const NODEITERATOR& leafnode) const{
            return leafnode.nrOfTriangles() ;
        };


	private:
        rw::geometry::TriMesh::Ptr _triangles;
	};

}
}

#endif /* BVTREE_HPP_ */
