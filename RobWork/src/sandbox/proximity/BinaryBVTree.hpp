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


template<class BVNODE>
class NodeIterator: public BVTreeNode<NodeIterator<BVNODE>, typename BVNODE::BVType>
{
public:
    typedef BVNODE BVNode;
    typedef typename BVNODE::BVType BVType;

    //! @brief constructor
    NodeIterator():node(NULL),_depth(0){};
    NodeIterator(BVNODE* n, unsigned char dep):node(n),_depth(dep){};

    inline const BVType& bv() const { return node->bv(); };
    inline bool leaf() const { return node->isLeaf(); };
    inline NodeIterator left() const { return NodeIterator( *node->left(), _depth+1 ); };
    inline NodeIterator right() const { return NodeIterator( *node->right(), _depth+1 ); };
    inline unsigned char depth() const { return _depth; };

    BVNODE *node;
    unsigned char _depth;
};

/**
 * @brief this node class stores the bv implicitly and use explicit pointers to its child nodes
 */
template<class BV>
class PtrNode {
public:
    typedef BV BVType;

    PtrNode():_size(0){
        _data._children._left = NULL;
        _data._children._right = NULL;
    }

    PtrNode(const BV& bv):
        _bv(bv),_size(0)
    {
        _data._children._left = NULL;
        _data._children._right = NULL;
    }

    virtual ~PtrNode(){
        if(_data._children._left)
            delete _data._children._left;
        if(_data._children._right)
            delete _data._children._right;
    }

    //! @brief get the OBB of this node
    BV& bv() {return _bv;}
    size_t& primIdx() {return _data._primIdxArray._primIdx;}
    int nrOfPrims() {return (int)_size;}
    void setNrOfPrims(int size){_size = (unsigned char)size;};

    PtrNode** left(){return &_data._children._left;};
    //void setleft(Node* left){_data._children._left = left;};
    PtrNode** right(){return &_data._children._right;};
    //void setright(Node* right){_data._children._right = right;};

    bool isLeaf(){ return _size>0;}

private:
    BV _bv;

    union {
        struct {
            PtrNode *_left;
            PtrNode *_right;
        } _children;
        struct {
            size_t _primIdx; // only used if its a leaf node
        } _primIdxArray;
    } _data;

    unsigned char _size;
};

//typedef PrtNode<OBB<double> > OBBPtrNodeD;
//typedef PrtNode<OBB<float> > OBBPtrNodeF;

	/**
	 * @brief a generic pointer based tree structure. This is not the most efficient
	 * structure for bounding volume trees. though it is quite generic and easy to work with
	 * becauseof its pointer based structure.
	 *
	 */
	template <class BV>
	class BinaryBVTree : public BVTree<NodeIterator<PtrNode<BV> > > {
	public:


		typedef BV BVType;
		typedef typename BV::value_type value_type;

		//typedef Node<BV> BVNode;

        typedef NodeIterator< PtrNode<BV> > iterator;
        typedef NodeIterator< PtrNode<BV> > node_iterator;
        typedef PtrNode<BV> Node;

		iterator getIterator() const { return iterator(); };


	public:
		//! @brief constructor
		BinaryBVTree(){}

		//! @brief constructor
		//BinaryBVTree(const BV& bv):_root(bv){}

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

		NodeIterator<Node> getRootIterator() const {
                return NodeIterator<Node>(_root, 0);
		};

		Node** getRoot(){return &_root;};

		int countNodes(){
			int count = 0;
			std::stack<Node*> children;
			children.push(_root);
			while(!children.empty()){
				Node *parent = children.top();
				children.pop();
				if(parent==NULL)
					continue;
				//std::cout << "parent size:" << (int)parent->nrOfPrims() << std::endl;
				count++;
				if(!parent->isLeaf()){

					children.push(*parent->left());
					children.push(*parent->right());
				}
			}
			return count;
		}

		int getMaxTrisPerLeaf() const{return 2;};

	private:

		Node *_root;
		std::vector<size_t> _leafIndexes;
	};


	typedef BinaryBVTree<rw::geometry::OBB<> > BinaryOBBPtrTreeD;
	typedef BinaryBVTree<rw::geometry::OBB<float> > BinaryOBBPtrTreeF;

}
}

#endif /* BINARYTREE_HPP_ */
