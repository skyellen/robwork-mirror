/*
 * BVTree.hpp
 *
 *  Created on: 05-02-2009
 *      Author: jimali
 */

#ifndef BVTREE_HPP_
#define BVTREE_HPP_

#include <sandbox/geometry/OBB.hpp>


namespace rw {
namespace proximity {

	/**
	 * @brief this implementation defines a BVTree structure that use
	 * an indexed based storage representation. The implementation does not include functionality
	 * for building a BVTree. It is a data structure for accessing nodes in a bounding volume tree.

	 * Specifically an array representation structured in a breadth first level-by-level
	 * Fashion.
	 *
	 *  Use this structure for balanced trees and for trees with static
	 * content.
	 *
	 */
	template <class BVTYPE>
	class BVTree {
	public:
		struct BVNode {
			BVNode():_isLeaf(-1),initialized(false){} // the node is invalid
			BVNode(BVTYPE &bv, char isleaf):_bv(bv),_isLeaf(isleaf),initialized(true){}
			inline bool isLeaf() const { return _isLeaf == 1; };
			inline bool isValid() const { return _isLeaf != -1; };

			BVTYPE _bv;
			char _isLeaf;
			bool initialized;
		};

		typedef BVTYPE BVType;

		virtual ~BVTree(){
			delete _bvnodes;
		};

		inline int getN(){
			return _N;
		}

		inline int getTrisPerLeaf(){
			return _trisPerLeaf;
		}

		inline int getRootIdx(){
			return 0;
		}

		inline int getChildIdx( int parentIdx, int childNr ){
			return _N*parentIdx + childNr + 1;
		};

		inline bool isLeafIdx(int nodeIdx){
			return (_firstLeafIdx<= nodeIdx) && (nodeIdx <=_lastLeafIdx);
		}

		inline bool hasChildIdx( int parentIdx, int childNr ){
			return !isLeafIdx( parentIdx );
		}

		inline BVNode& get( int idx ){
			if( has(idx) )
				return (*_bvnodes)[idx];
			RW_THROW("Invalid idx!!!");
		}

		inline bool has( int idx){
			return (0<= idx) && (idx<_bvnodes->size());
		}

		inline BVNode& getChild( int parentIdx, int childNr ){
			if( hasChildIdx(parentIdx, 0) )
				return (*_bvnodes)[ getChildIdx(parentIdx, 0) ];
			RW_THROW("No such child!");
		}

		/**
		 * @brief returns a pointer to the first child in the array
		 * of children
		 */
		inline BVNode* getChildren( int parentIdx ){
			if( hasChildIdx(parentIdx, 0) )
				return (*_bvnodes)[ getChildIdx(parentIdx, 0) ];
			return NULL;
		}

		inline std::vector<BVNode>& getNodes(){ return *_bvnodes; };

		/**
		 * @brief gets a pointer to the first triangle of node with
		 * idx \b nodeIdx
		 */
		inline int getTriChildIdx(int nodeIdx, int childNr=0){
			return (nodeIdx-_firstLeafIdx)*_trisPerLeaf + childNr;
		}

		/**
		 * @brief gets a pointer to the first triangle of node with
		 * idx \b nodeIdx
		 */
		/*
		inline int* getTriChild( int nodeIdx,int childNr=0 ){
			bool isLeaf = isLeafIdx(nodeIdx);

			if (!isLeaf) {
				std::cout << "nodeIdx = " << nodeIdx << std::endl;
				std::cout << "_firstLeafIdx = " << _firstLeafIdx << std::endl;
				std::cout << "getTriChildIdx() = " << getTriChildIdx(nodeIdx) << std::endl;
			}
			RW_ASSERT(isLeaf);

			// return &( (*_triIndexes)[ (nodeIdx-_firstLeafIdx)*_trisPerLeaf ] );
			int index = getTriChildIdx(nodeIdx, childNr);
			RW_ASSERT( 0<=index && index<_triIndexes->size() );
			return &( (*_triIndexes)[ index ] );
		}
		*/

		inline int* getTri(int idx){
			return &( (*_triIndexes)[ idx ] );
		}

		inline size_t getNrLeafs(){ return _lastLeafIdx-_firstLeafIdx+1; };

		/**
		 * @brief builds a balanced BVTree using the median
		 */
		//static BVTree<T>* buildBalancedBinaryTree(const rw::geometry::IndexedTriMesh<T>& imesh);

		//static BVTree<T>* buildBalancedNaryTree(int n, const rw::geometry::IndexedTriMesh<T>& imesh);

		/**
		 * @brief creates a
		 */
		BVTree(std::vector<BVNode> *bvnodes, int n,
			   std::vector<int> *triIdx, int trisPerLeaf,
			   int depth):
			_bvnodes(bvnodes),
			_triIndexes(triIdx),
			_size(bvnodes->size()),
			_N(n),
			_trisPerLeaf(trisPerLeaf),
			_depth( depth ),
			_firstLeafIdx( std::max((int)std::floor( std::pow((double)_N, _depth))-1,0)),
			_lastLeafIdx(  std::max((int)std::floor( std::pow((double)_N, _depth+1))-2,0))
			//_firstLeafIdx( (int)std::ceil( std::pow((double)n,_depth-1)) ),
			//_lastLeafIdx( (int)std::ceil( std::pow((double)n,_depth)) )

		{

			std::cout << "Calc size: " << (bvnodes->size()-1)*trisPerLeaf << std::endl;
			std::cout << "real size: " << triIdx->size() << std::endl;
			std::cout << "Depth is: " << _depth << std::endl;
			std::cout << "firstLeafIdx: " << _firstLeafIdx<< std::endl;
			std::cout << "lastLeafIdx: " << _lastLeafIdx<< std::endl<<std::endl;
		}

	protected:

	private:
		std::vector<BVNode> *_bvnodes;
		std::vector<int> *_triIndexes;
		const int _size;
		const int _N, _trisPerLeaf;
		const int _depth;
		const int _firstLeafIdx, _lastLeafIdx;
	};

	typedef BVTree<rw::geometry::OBB<float> > OBBfTree;

}
}

#endif /* BVTREE_HPP_ */
