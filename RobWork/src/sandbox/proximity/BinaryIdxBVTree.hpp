/*
 * BinaryTree.hpp
 *
 *  Created on: 05/08/2010
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_BINARYTREE_HPP_
#define RW_PROXIMITY_BINARYTREE_HPP_

#include <stack>
#include "BVTree.hpp"
namespace rw {
namespace proximity {




    template<class BVTREE, class BV>
    class IdxNodeIterator: public BVTreeNode<BVTREE, BV>
    {
    public:
        typedef typename BVTREE::Node BVNode;
        typedef BV BVType;

        //! @brief constructor
        IdxNodeIterator():_nodeIdx(-1),_depth(0){};
        IdxNodeIterator(BVTREE* tree, int nodeidx, unsigned char dep):
            _tree(tree),_nodeIdx(nodeidx),_depth(dep){};

        inline const BVType& bv() const { return _tree->bv(_nodeIdx); };
        inline bool leaf() const { return _tree->isLeaf(_nodeIdx); };
        inline IdxNodeIterator left() const { return IdxNodeIterator( _tree, _tree->left(_nodeIdx), _depth+1 ); };
        inline IdxNodeIterator right() const { return IdxNodeIterator( _tree, _tree->right(_nodeIdx), _depth+1 ); };
        inline unsigned char depth() const { return _depth; };

        BVTREE *_tree;
        int _nodeIdx;
        unsigned char _depth;
    };


//typedef PrtNode<OBB<double> > OBBPtrNodeD;
//typedef PrtNode<OBB<float> > OBBPtrNodeF;

	/**
	 * @brief a generic pointer based tree structure. This is not the most efficient
	 * structure for bounding volume trees. though it is quite generic and easy to work with
	 * because of its pointer based structure.
	 *
	 */
	template <class BV>
	class BinaryIdxBVTree : public BVTree<IdxNodeIterator<BinaryIdxBVTree<BV>, BV> > {
	public:


		typedef BV BVType;
		typedef typename BV::value_type value_type;


		/**
		 * @brief this node class stores the bv implicitly and use explicit pointers to its child nodes
		 */
		//template<class BV>
		class IdxNode {
		public:
		    typedef BV BVType;

		    IdxNode(){
		        _left = -1;
		        _right = -1;
		    }

		    IdxNode(const BV& bv):
		        _bv(bv)
		    {
		        _left = -1;
		        _right = -1;
		    }

		    virtual ~IdxNode(){}

		    //! @brief get the OBB of this node
		    BV& bv() {return _bv;}
		    size_t& primIdx() {return _right;}
		    int nrOfPrims() {return -1*(_left+2);}
		    void setNrOfPrims(int size){_left = -2-size;};

		    //IdxNode** left(){return &_data._children._left;};
		    //void setleft(Node* left){_data._children._left = left;};
		    //IdxNode** right(){return &_data._children._right;};
		    //void setright(Node* right){_data._children._right = right;};

		    bool isLeaf(){ return _left<-1;}

		private:
		    BV _bv;

		    int _left;
		    int _right; // when left is <-1 then right points to the primitives start index
		};

		//typedef Node<BV> BVNode;

        typedef IdxNodeIterator< BinaryIdxBVTree<BV>, IdxNode > iterator;
        typedef IdxNodeIterator< BinaryIdxBVTree<BV>, IdxNode > node_iterator;
        typedef IdxNode Node;

		iterator getIterator() const { return iterator(); };

	public:
		//! @brief constructor
		BinaryIdxBVTree()
		{
		    _nodes.reserve(300);
		}

		//! @brief constructor
		Node* createNode(const BV& bv){
			return new Node(bv);
		};

		Node* createNode(){
			return new Node();
		};

		void setLeafPrimitives(Node* node, size_t primStartIdxs){
			if( !node->isLeaf() )
				RW_THROW("Not a leaf node!");

			node->primIdx() = _leafIndexes.size();

		}

		node_iterator getRootIterator() const {
                return node_iterator(this, 0, 0);
		};

		Node* getRoot(){return &_nodes[0];};

		int countNodes(){
		    return _nodes.size();
		}

		int getMaxTrisPerLeaf() const{return 2;};

		inline int left(int idx){
		    return _nodes[idx].left();
		}

		inline int right(int idx){
		    return _nodes[idx].right();
		}

		inline bool isLeaf(int idx){
		    return _nodes[idx].isLeaf();
		}

		inline BV& bv(int idx){
		    return _nodes[idx].bv();
		}

	private:

		std::vector<Node> _nodes;
		std::vector<size_t> _leafIndexes;
	};


	typedef BinaryIdxBVTree<rw::geometry::OBB<> > BinaryOBBIdxTreeD;
	typedef BinaryIdxBVTree<rw::geometry::OBB<float> > BinaryOBBIdxTreeF;

}
}

#endif /* BINARYTREE_HPP_ */
