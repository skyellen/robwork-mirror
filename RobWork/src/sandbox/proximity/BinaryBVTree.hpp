/*
 * BinaryTree.hpp
 *
 *  Created on: 05/08/2010
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_BINARYBVTREE_HPP_
#define RW_PROXIMITY_BINARYBVTREE_HPP_

#include <stack>
#include "BVTree.hpp"
namespace rw {
namespace proximity {


    /**
     * @brief this node class stores the bv implicitly and use explicit pointers to its child nodes
     */
    template<class BV>
    class PtrNode {
    public:
        typedef BV BVType;

        /**
         * @brief an iterator for the PtrNode
         */
        class NodeIterator: public BVTreeNode<typename PtrNode<BV>::NodeIterator, BV>
        {
        public:
            typedef PtrNode<BV> BVNode;
            typedef BV BVType;

            //! @brief constructor
            NodeIterator():node(NULL),_depth(0){};
            NodeIterator(PtrNode<BV>* n, unsigned char dep):node(n),_depth(dep){};

            inline const BVType& bv() const { return node->bv(); };
            inline bool leaf() const { return node->isLeaf(); };
            inline NodeIterator left() const { return NodeIterator( node->left(), _depth+1 ); };
            inline NodeIterator right() const { return NodeIterator( node->right(), _depth+1 ); };
            inline unsigned char depth() const { return _depth; };
            inline bool hasLeft(){ return node->left()!=NULL; }
            inline bool hasRight(){ return node->right()!=NULL; }
            PtrNode<BV> *node;
            unsigned char _depth;
        };

    public:

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

        PtrNode* left(){
            return _data._children._left;
        };

        PtrNode* right(){
            return _data._children._right;
        };

        void setLeft(PtrNode*  left){_data._children._left = left;};
        void setRight(PtrNode* right){_data._children._right = right;};

        bool isLeaf(){ return _size>0 || ((left()==NULL) && (right()==NULL));}

    private:
        BV _bv;

        struct {
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

	/**
	 * @brief a generic pointer based tree structure. This is not the most efficient
	 * structure for bounding volume trees. though it is quite generic and easy to work with
	 * because of its pointer based structure.
	 *
	 */
	template <class BV>
	class BinaryBVTree : public BVTree<typename PtrNode<BV>::NodeIterator> {
	public:


		typedef BV BVType;
		typedef typename BV::value_type value_type;

		//typedef Node<BV> BVNode;

        typedef typename PtrNode<BV>::NodeIterator iterator;
        typedef typename PtrNode<BV>::NodeIterator node_iterator;
        typedef PtrNode<BV> Node;

		iterator getIterator() const { return iterator(); };


	public:
		//! @brief constructor
		BinaryBVTree():_root(NULL){}

		//! @brief constructor
		//BinaryBVTree(const BV& bv):_root(bv){}

		node_iterator createLeft( node_iterator parent){
            parent.node->setLeft( new Node() );
            return node_iterator(parent.node->left(), parent.depth()+1);
        }

		node_iterator createRight( node_iterator parent ){
		    parent.node->setRight( new Node() );
		    return node_iterator(parent.node->right(), parent.depth()+1);
		}

		node_iterator createRoot(){
		    if( _root ==NULL){
		        _root = new Node();
		    }
		    return node_iterator(_root,0);
		}

        void setBV(const BVType& bv, node_iterator node){
            node.node->bv() = bv;
        }

        void setNrOfPrims(int size, node_iterator node){
            node.node->setNrOfPrims(size);
        }

        void setPrimIdx(int primIdx, node_iterator node){
            node.node->primIdx() = primIdx;
        }

        void compile(){

        }


		void setLeafPrimitives(Node* node, size_t primStartIdxs){
			if( !node->isLeaf() )
				RW_THROW("Not a leaf node!");

			node->primIdx() = _leafIndexes.size();

		}

		node_iterator getRootIterator() const {
                return node_iterator(_root, 0);
		};

		Node* getRoot(){return _root;};

		std::pair<int,int> countNodes(){
			int ncount = 0, lcount = 0;

			std::stack<Node*> children;
			children.push(_root);
			while(!children.empty()){
			    //std::cout << count << std::endl;
				Node *parent = children.top();
				children.pop();
				if(parent==NULL)
					continue;
				//std::cout << "parent size:" << (int)parent->nrOfPrims() << std::endl;
				//std::cout << parent->bv().getHalfLengths() << std::endl;

				if(!parent->isLeaf()){
				    ncount++;
					children.push(parent->left());
					children.push(parent->right());
				} else {
				    lcount++;
				}
			}
			return std::make_pair(ncount,lcount);
		}

        void print(){
            std::stack<std::pair<Node*, int> > children;
            children.push(std::make_pair(_root,0));
            int id=1;
            std::cout << "digraph BinaryBVTree {\n";
            while(!children.empty()){

                std::pair<Node*,int> parent = children.top();
                children.pop();
                if(parent.first==NULL)
                    continue;
                //std::cout << "parent size:" << (int)parent->nrOfPrims() << std::endl;
                //std::cout << parent->bv().getHalfLengths() << std::endl;
                //std::cout << "\t" << parent.second << " [label=\"" << id <<"\n";
                if(!parent.first->isLeaf()){
                    id++;
                    std::cout << "\t" << parent.second << "->" << id <<"\n";
                    std::cout << "\t" << parent.second << "[label=\"" << parent.first->bv().calcVolumne() <<"\"]\n";
                    if(parent.first->left()!=NULL){
                        children.push(std::make_pair(parent.first->left(), id));
                    } else {
                        std::cout << "\t" << parent.second << " [shape=point]\n";
                    }
                    id++;
                    std::cout << "\t" << parent.second << "->" << id <<"\n";
                    if(parent.first->right()!=NULL){
                        children.push(std::make_pair(parent.first->right(),id));
                    } else {
                        std::cout << "\t" << parent.second << " [shape=point]\n";
                    }
                } else {
                    std::cout << "\t" << parent.second << " [shape=box]\n";
                    std::cout << "\t" << parent.second << " [label=\""<< parent.first->primIdx() <<"\"]\n";
                }
            }
            std::cout << "}\n";

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
