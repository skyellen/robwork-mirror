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

#ifndef RW_PROXIMITY_BVTREE_HPP_
#define RW_PROXIMITY_BVTREE_HPP_

#include <rw/geometry/OBB.hpp>


namespace rw {
namespace proximity {

    template<class PRIM>
    struct PrimArrayAccessor {
        typedef PRIM PRIMType;
        virtual ~PrimArrayAccessor() {}
        virtual void getPrimitive(size_t pidx, PRIM& prim) const = 0;
        virtual size_t getSize() const = 0;
        //virtual const PRIM& getPrimitive(size_t pidx) const = 0;
    };

    template<class DERIVED, class BV>
    struct BVTreeIterator {
        typedef BV BVType;
        //! @brief constructor

        inline DERIVED* downcast(){ return static_cast<const DERIVED*>(this); }
        inline const DERIVED* downcast() const { return static_cast<const DERIVED*>(this); }

        inline int getId() const { return downcast()->getId(); };
        inline const BVType& getBV() const { return downcast()->bv(); };
        inline bool isLeaf() const { return downcast()->leaf(); };
        inline DERIVED left() const { return downcast()->left(); };
        inline DERIVED right() const { return downcast()->right(); };
        inline bool hasRight() const { return downcast()->hasRight(); };
        inline bool hasLeft() const { return downcast()->hasLeft(); };
        inline unsigned char depth() const { return downcast()->depth(); };

        inline size_t primitiveIdx() const {return downcast()->primitiveIdx();}
        inline size_t nrOfPrimitives() const { return downcast()->nrOfPrimitives();}

        //inline std::vector<PRIMType> getPrimitives() const {
        //    return downcast()->getPrimitives();
        //};
    };

	/**
	 * @brief this implementation defines a BVTree structure that use
	 * an indexed based storage representation. The implementation does not include functionality
	 * for building a BVTree. It is a data structure for accessing nodes in a bounding volume tree.
	 *
	 */
#ifdef bubmumb
    template<class NODEITERATOR>
	class BVTree {
	public:
		typedef typename NODEITERATOR::BVType BVType;
		typedef typename NODEITERATOR::PRIMType PRIMType;
		typedef typename BVType::value_type value_type;
		typedef NODEITERATOR Node;


		BVTree(rw::geometry::TriMesh::Ptr triangles):
		    _triangles(triangles)
		{}

		virtual ~BVTree(){};

		virtual NODEITERATOR getRootIterator() const = 0;//{ return static_cast<DERIVED*>(this)->getRoot(); }
		virtual int getMaxTrisPerLeaf() const = 0;

		virtual NODEITERATOR createLeft( NODEITERATOR parent) = 0;
		virtual NODEITERATOR createRight( NODEITERATOR parent ) = 0;
		virtual NODEITERATOR createRoot() = 0;

		virtual void setBV(const BVType& bv, NODEITERATOR node) = 0;
        virtual void setNrOfPrims(int size, NODEITERATOR node) = 0;
        virtual void setPrimIdx(int primIdx, NODEITERATOR node) = 0;

        virtual void compile() = 0;

        /**
         * @brief get triangle nr \b triNr that the BVNode \b leafnode is bounding. The result is
         * set in triangle \b tridst and the index of the triangle is returned
         * @param leafnode [int] the leaf containing triangles
         * @param tridst [out] the container for the triangle
         * @param triNr [in] the triangle nr
         * @return global index of triangle
         */
        inline int getTriangle(const NODEITERATOR& leafnode,rw::geometry::Triangle<value_type>& dst, size_t triNr) const {
            RW_ASSERT(leafnode.nrOfTriangles()>0);
            size_t idx = leafnode.triangleIdx();
            //std::cout << "Idx: " << idx+triNr << "  " << idx<<"+"<<triNr<< "\n";
            _triangles->getTriangle(idx+triNr, dst);
            //std::cout << dst[0] << "\n";
            return idx+triNr;
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
#endif

    template<class DERIVED>
    class BVTree {
    public:
        typedef typename Traits<DERIVED>::BVType BVType;
        typedef typename Traits<DERIVED>::PRIMType PRIMType;
        typedef typename BVType::value_type value_type;
        typedef typename Traits<DERIVED>::NodeIterator NodeIterator;
        typedef typename Traits<DERIVED>::Node Node;





    public:

        /**
         * @brief Constructor. The BVTree takes ownership of the \b primAccessor .
         * @param primAccessor
         */
        BVTree(PrimArrayAccessor<PRIMType>* primAccessor):_pAccessor(primAccessor)
        {
        }

        //! @brief Destructor.
        virtual ~BVTree(){
        	delete _pAccessor;
        }

        virtual NodeIterator getRootIterator() const = 0;//{ return static_cast<DERIVED*>(this)->getRoot(); }
        virtual int getMaxTrisPerLeaf() const = 0;

        virtual NodeIterator createLeft( NodeIterator parent) = 0;
        virtual NodeIterator createRight( NodeIterator parent ) = 0;
        virtual NodeIterator createRoot() = 0;

        virtual void setBV(const BVType& bv, NodeIterator node) = 0;
        virtual void setNrOfPrims(int size, NodeIterator node) = 0;
        virtual void setPrimIdx(int primIdx, NodeIterator node) = 0;

        virtual void optimize() = 0;

        /**
         * @brief get triangle nr \b triNr that the BVNode \b leafnode is bounding. The result is
         * set in triangle \b tridst and the index of the triangle is returned
         * @param leafnode [int] the leaf containing triangles
         * @param tridst [out] the container for the triangle
         * @param triNr [in] the triangle nr
         * @return global index of triangle
         */
        inline int getPrimitive(const NodeIterator& leafnode, PRIMType& dst, size_t triNr) const {
            RW_ASSERT(leafnode.nrOfPrimitives()>0);
            size_t idx = leafnode.primitiveIdx();
            //std::cout << "Idx: " << idx+triNr << "  " << idx<<"+"<<triNr<< "\n";
            _pAccessor->getPrimitive(idx+triNr, dst);
            //std::cout << dst[0] << "\n";
            return (int)(idx+triNr);
        }

        inline size_t getNrPrimitives(const NodeIterator& leafnode) const{ return leafnode.nrOfPrimitives() ; };

        /*
        inline int getPrimitive(const NodeIterator& leafnode, PRIMType& dst, size_t triNr) const {
            return ((DERIVED)this)->getPrimitive(leafnode, dst, triNr);
        }
        */

    private:
        const PrimArrayAccessor<PRIMType>* const _pAccessor;
    };


}
}

#endif /* BVTREE_HPP_ */
