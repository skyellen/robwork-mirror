/*
 * BinaryTree.hpp
 *
 *  Created on: 05/08/2010
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_BINARYTREE_HPP_
#define RW_PROXIMITY_BINARYTREE_HPP_

namespace rw {
namespace proximity {

	/**
	 * @brief a generic pointer based tree structure. This is not the most efficient
	 * structure for bounding volume trees. though it is quite generic and easy to work with
	 * becauseof its pointer based structure.
	 *
	 */
	template <class BV>
	class BinaryBVTree {
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

			~Node(){
				if(_data._children._left)
					delete _data._children._left;
				if(_data._children._right)
					delete _data._children._right;
			}

			//! @brief get the OBB of this node
			BV& bv() {return _bv;}
			size_t& primIdx() {return _data._primIdxArray._primIdx;}
			unsigned char& nrOfPrims() {return _size;}

			Node*& left(){return _data._children._left;};
			Node*& right(){return _data._children._right;};

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
	public:
		//! @brief constructor
		BinaryBVTree(){}

		//! @brief constructor
		BinaryBVTree(const BV& bv):_root(bv){}

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

		Node* getRoot(){return &_root;};

	private:

		Node _root;
		std::vector<size_t> _leafIndexes;
	};
}
}

#endif /* BINARYTREE_HPP_ */
