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



	/**
	 * @brief a generic pointer based tree structure. This is not the most efficient
	 * structure for bounding volume trees. though it is quite generic and easy to work with
	 * becauseof its pointer based structure.
	 *
	 */
	template <class BV>
	class BinaryBVTree : public BVTree {
	public:

		struct Node {
		public:
			Node():_size(0){
				_data._children._left = NULL;
				_data._children._right = NULL;
			}

			Node(const BV& bv):
				_bv(bv),_size(0)
			{
				_data._children._left = NULL;
				_data._children._right = NULL;
			}

			virtual ~Node(){
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

			Node** left(){return &_data._children._left;};
			//void setleft(Node* left){_data._children._left = left;};
			Node** right(){return &_data._children._right;};
			//void setright(Node* right){_data._children._right = right;};

			bool isLeaf(){ return _size>0;}

		private:
			BV _bv;

			union {
				struct {
					Node *_left;
					Node *_right;
				} _children;
				struct {
					size_t _primIdx; // only used if its a leaf node
				} _primIdxArray;
			} _data;

			unsigned char _size;
		};

		struct nodeiterator : public BVTree::Node<nodeiterator, BV> {
			typedef BV BVType;
			//! @brief constructor
			nodeiterator():node(NULL),depth(0){};
			nodeiterator(Node* n, unsigned char dep):node(n),depth(dep){};

			inline const BV& bv() const { return node->bv(); };
			inline bool leaf() const { return node->isLeaf(); };
			inline nodeiterator left() const { return nodeiterator( *node->left() ); };
			inline nodeiterator right() const { return nodeiterator( *node->right() ); };
			inline unsigned char depth() const { return depth; };

			Node *node;
			unsigned char depth;
		};

		typedef BV BVType;
		typedef typename BV::value_type value_type;
		typedef nodeiterator iterator;
		typedef nodeiterator BVNode;

		nodeiterator getIterator() const { return nodeiterator(); };


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
}
}

#endif /* BINARYTREE_HPP_ */
