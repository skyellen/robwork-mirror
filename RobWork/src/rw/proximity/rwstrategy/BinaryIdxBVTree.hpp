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

#ifndef RW_PROXIMITY_BINARYIDXTREE_HPP_
#define RW_PROXIMITY_BINARYIDXTREE_HPP_

//#include <stack>
//#include "BVTree.hpp"
namespace rw {
namespace proximity {

#ifdef xkdfnslafn
    /**
     * @brief this node class stores the bv implicitly and use explicit pointers to its child nodes
     */
    template<class BVTREE, class BV, class PRIM>
    class IdxNode {
    public:
        typedef BV BVType;
        typedef PRIM PRIMType;

        class NodeIterator: public BVTreeIterator<typename IdxNode<BVTREE,BV,PRIM>::NodeIterator, BV, PRIM>
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
            inline size_t primitiveIdx() const {return _tree->primIdx(_nodeIdx);}
            inline size_t nrOfPrimitives() const { return _tree->nrOfPrims(_nodeIdx);}

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
        int primIdx() const {return _right;}
        void setPrimIdx(int pidx){
            _right = pidx;
            _left = -2; // indicates that this is a leaf node
        }
        int nrOfPrims() const {return -1*(_left+2);}
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
	template <class BV, class PRIM>
	class BinaryIdxBVTree : public BVTree<typename IdxNode<BinaryIdxBVTree<BV, PRIM> >::NodeIterator > {
	public:
		typedef BV BVType;
		typedef typename BV::value_type value_type;

		typedef IdxNode< BinaryIdxBVTree<BV, PRIM>, BV, PRIM> Node;
        typedef typename Node::NodeIterator iterator;
        typedef typename Node::NodeIterator node_iterator;

		iterator getIterator() const { return iterator(); };

	public:
		//! @brief constructor
		BinaryIdxBVTree(PrimArrayAccessor<PRIM>* paccessor):
		    BVTree<node_iterator>(paccessor)
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

        inline size_t nrOfPrims(int idx) const {
            return _nodes[idx].nrOfPrims();
        }

        inline size_t primIdx(int idx) const {
            return _nodes[idx].primIdx();
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
#endif

	//typedef BinaryIdxBVTree<rw::geometry::OBB<> > BinaryOBBIdxTreeD;
	//typedef BinaryIdxBVTree<rw::geometry::OBB<float> > BinaryOBBIdxTreeF;

}
}

#endif /* BINARYTREE_HPP_ */
