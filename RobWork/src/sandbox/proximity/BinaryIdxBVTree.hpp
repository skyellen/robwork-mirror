/*
 * BinaryTree.hpp
 *
 *  Created on: 05/08/2010
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_BINARYIDXTREE_HPP_
#define RW_PROXIMITY_BINARYIDXTREE_HPP_

#include <stack>
#include "BVTree.hpp"
namespace rw {
namespace proximity {


    /**
     * @brief this node class stores the bv implicitly and use explicit pointers to its child nodes
     */
    template<class BVTREE, class BV>
    class IdxNode {
    public:
        typedef BV BVType;

        class NodeIterator: public BVTreeNode<typename IdxNode<BVTREE,BV>::NodeIterator, BV>
        {
        public:
            typedef IdxNode BVNode;
            typedef BV BVType;

            //! @brief constructor
            NodeIterator():_nodeIdx(-1),_depth(0){};
            NodeIterator(const BVTREE* tree, int nodeidx, unsigned char dep):
                _tree(tree),_nodeIdx(nodeidx),_depth(dep){};

            inline const BVType& bv() const { return _tree->bv(_nodeIdx); };
            inline bool leaf() const { return _tree->isLeaf(_nodeIdx); };
            inline NodeIterator left() const { return NodeIterator( _tree, _tree->left(_nodeIdx), _depth+1 ); };
            inline NodeIterator right() const { return NodeIterator( _tree, _tree->right(_nodeIdx), _depth+1 ); };
            inline unsigned char depth() const { return _depth; };

            const BVTREE *_tree;
            int _nodeIdx;
            unsigned char _depth;
        };


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
        const BV& bv() const {return _bv;}
        int primIdx() {return _right;}
        void setPrimIdx(int pidx){
            _right = pidx;
            _left = -2; // indicates that this is a leaf node
        }
        int nrOfPrims() {return -1*(_left+2);}
        void setNrOfPrims(int size){_left = -2-size;};

        int left() const {return _left;};
        int right() const {return _right;};
        void setLeft(int left){_left = left;};
        void setRight(int right){_right = right;};

        bool isLeaf() const { return _left<-1;}

    private:
        BV _bv;

        int _left;
        int _right; // when left is <-1 then right points to the primitives start index
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
	class BinaryIdxBVTree : public BVTree< typename IdxNode<BinaryIdxBVTree<BV>, BV>::NodeIterator > {
	public:
		typedef BV BVType;
		typedef typename BV::value_type value_type;

		typedef IdxNode< BinaryIdxBVTree<BV>, BV> Node;
        typedef typename Node::NodeIterator iterator;
        typedef typename Node::NodeIterator node_iterator;

		iterator getIterator() const { return iterator(); };

	public:
		//! @brief constructor
		BinaryIdxBVTree()
		{
		    _nodes.reserve(300);
		}

		node_iterator getRootIterator() const {
                return node_iterator(this, 0, 0);
		};

		Node* getRoot(){return &_nodes[0];};

		int countNodes(){
		    return _nodes.size();
		}

		int getMaxTrisPerLeaf() const{return 2;};

		inline int left(int idx) const {
		    return _nodes[idx].left();
		}

		inline int right(int idx) const {
		    return _nodes[idx].right();
		}

		inline bool isLeaf(int idx) const {
		    return _nodes[idx].isLeaf();
		}

		inline BV& bv(int idx){
		    return _nodes[idx].bv();
		}

        inline const BV& bv(int idx) const {
            return _nodes[idx].bv();
        }

        node_iterator createLeft( node_iterator parent){
            _nodes.push_back( Node() );
            int nidx = _nodes.size()-1;
            _nodes[ parent._nodeIdx ].setLeft( nidx );
            return node_iterator(this, _nodes.size()-1 , parent.depth()+1);
        }

        node_iterator createRight( node_iterator parent ){
            _nodes.push_back( Node() );
            int nidx = _nodes.size()-1;
            _nodes[ parent._nodeIdx ].setRight( nidx );
            return node_iterator(this, _nodes.size()-1 , parent.depth()+1);
        }

        node_iterator createRoot(){
            if(_nodes.size()==0){
                _nodes.push_back( Node() );
            }
            return node_iterator(this, 0 , 0);
        }

        void setBV(const BVType& bv, node_iterator node){
            this->bv(node._nodeIdx) = bv;
        }

        void setNrOfPrims(int size, node_iterator node){
            _nodes[node._nodeIdx].setNrOfPrims( size );
        }

        void setPrimIdx(int primIdx, node_iterator node){
            _nodes[node._nodeIdx].setPrimIdx(primIdx);
        }

        void compile(){};

        Node* createNode(const BV& bv){
            _nodes.push_back( Node(bv) );
            return &_nodes.back();
        };

        Node* createNode(){
            _nodes.push_back( Node() );
            return &_nodes.back();
        };

	private:

		std::vector<Node> _nodes;
		std::vector<size_t> _leafIndexes;
	};


	typedef BinaryIdxBVTree<rw::geometry::OBB<> > BinaryOBBIdxTreeD;
	typedef BinaryIdxBVTree<rw::geometry::OBB<float> > BinaryOBBIdxTreeF;

}
}

#endif /* BINARYTREE_HPP_ */
