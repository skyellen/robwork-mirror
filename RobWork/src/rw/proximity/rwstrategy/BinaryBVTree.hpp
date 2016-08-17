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

#ifndef RW_PROXIMITY_BINARYBVTREE_HPP_
#define RW_PROXIMITY_BINARYBVTREE_HPP_

#include <stack>
#include "BVTree.hpp"
//#include <boost/tuple/tuple.hpp>

namespace rw {
namespace proximity {

    /**
     * @brief this node class stores the bv implicitly and use explicit pointers to its child nodes.
     * This is an inefficient storage method and for general usage the other types should be used.
     * However, there is only a small overhead for adding and deleting nodes so
     * for very dynamic uses this might still be applicable.
     */
    template<class BV, class PRIM>
    class BTPNode {
    public:
        typedef BV BVType;
        typedef PRIM PRIMType;

        BTPNode():_size(0){
            _data._children._left = NULL;
            _data._children._right = NULL;
            _data._primIdxArray._primIdx = 0;
        }

        BTPNode(const BV& bv):
            _bv(bv),_size(0)
        {
            _data._children._left = NULL;
            _data._children._right = NULL;
            _data._primIdxArray._primIdx = 0;
        }

        virtual ~BTPNode(){
        	if (!isLeaf()) {
        		if(_data._children._left != NULL)
        			delete _data._children._left;
        		if(_data._children._right != NULL)
        			delete _data._children._right;
        		_data._children._left = NULL;
        		_data._children._right = NULL;
        	}
        }

        //! @brief get the OBB of this node
        inline BV& bv() {return _bv;}
        inline size_t& primIdx() {return _data._primIdxArray._primIdx;}
        inline int nrOfPrims() {return (int)_size;}
        inline void setNrOfPrims(int size){_size = (unsigned char)size;};

        inline BTPNode* left(){ return _data._children._left; };
        inline BTPNode* right(){ return _data._children._right; };

        inline void setLeft(BTPNode*  left){
        	RW_ASSERT(!isLeaf());
            if(_data._children._left != NULL)
                delete _data._children._left;
            _data._children._left = left;
        }
        inline void setRight(BTPNode* right){
        	RW_ASSERT(!isLeaf());
            if(_data._children._right != NULL)
                delete _data._children._right;
        	_data._children._right = right;
        }

        inline bool isLeaf(){ return _size>0 /*|| ((left()==NULL) && (right()==NULL))*/;}


    public:
        /**
         * @brief an iterator for the PtrNode
         */
        class NodeIterator: public BVTreeIterator<typename BTPNode<BV,PRIM>::NodeIterator, BV>
        {
        public:
            typedef BTPNode<BV,PRIM> Node;

            //! @brief constructor
            NodeIterator():node(NULL),_depth(0){};
            NodeIterator(Node* n, unsigned char dep):node(n),_depth(dep){};

            inline const BVType& bv() const { return node->bv(); };
            inline bool leaf() const { return node->isLeaf(); };
            inline NodeIterator left() const { return NodeIterator( node->left(), _depth+1 ); };
            inline NodeIterator right() const { return NodeIterator( node->right(), _depth+1 ); };
            inline unsigned char depth() const { return _depth; };
            inline bool hasLeft() const { return node->left()!=NULL; }
            inline bool hasRight() const { return node->right()!=NULL; }
            //inline int getId() const { return (int)node; };

            inline size_t primitiveIdx() const {return node->primIdx();}
            inline size_t nrOfPrimitives() const { return node->nrOfPrims();}

            Node *node;
            unsigned char _depth;
        };


    private:
        BV _bv;

        union {
            struct {
                BTPNode *_left;
                BTPNode *_right;
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
	template <class BV, class PRIM>
	class BinaryBVTree : public BVTree< BinaryBVTree<BV,PRIM> > {
	public:
	    typedef BV BVType;
		typedef PRIM PRIMType;
		typedef typename BV::value_type value_type;
		typedef rw::common::Ptr<BinaryBVTree<BV,PRIM> > Ptr;
		typedef BTPNode<BV,PRIM> Node;
		typedef typename BTPNode<BV,PRIM>::NodeIterator NodeIterator;

	public:
		//! @brief constructor
		BinaryBVTree(PrimArrayAccessor<PRIM>* paccessor):
		    BVTree<BinaryBVTree<BV,PRIM> >(paccessor),
		    _root(NULL)
		{
		    //std::cout << "SIZE OF NODE: " << sizeof(Node) << std::endl;
		}

		//! @brief Destructor.
		~BinaryBVTree() {
			if (_root != NULL) {
				delete _root;
				_root = NULL;
			}
		}

		NodeIterator getIterator() const { return getRootIterator(); };

		NodeIterator createLeft( NodeIterator parent){
            parent.node->setLeft( new Node() );
            return NodeIterator(parent.node->left(), parent.depth()+1);
        }

		NodeIterator createRight( NodeIterator parent ){
		    parent.node->setRight( new Node() );
		    return NodeIterator(parent.node->right(), parent.depth()+1);
		}

		NodeIterator createRoot(){
		    if( _root ==NULL){
		        _root = new Node();
		    }
		    return NodeIterator(_root,0);
		}

        void setBV(const BVType& bv, NodeIterator node){
            node.node->bv() = bv;
        }

        void setNrOfPrims(int size, NodeIterator node){
            node.node->setNrOfPrims(size);
        }

        void setPrimIdx(int primIdx, NodeIterator node){
            node.node->primIdx() = primIdx;
        }

        void optimize(){
            using namespace rw::common;
            using namespace boost;
            /*
            // this copies all nodes into a memory friendlier structure
            std::pair<int,int> nrNodes = countNodes();
            rw::common::Ptr<std::vector<Node> > nodes = ownedPtr( new std::vector<Node>(nrNodes.first+nrNodes.second) );
            // now we start with the root node and add all nodes in a preorder traversal fashion.
            int nodeIndex = 0;
            // tuple <new parent, child, isLeft>
            std::stack<boost::tuple<Node*, Node*, bool> > children;
            children.push(boost::make_tuple((Node*)NULL, _root, true) );
            while(!children.empty()){
                //std::cout << count << std::endl;
                boost::tuple<Node*, Node*, bool> node = children.top();
                children.pop();

                if(get<1>( node )==NULL)
                    continue;

                // copy node to next index
                (*nodes)[nodeIndex] = *node.get<1>();
                // change parent to point to this new child
                if(node.get<0>()!=NULL){
                    if(node.get<2>())// then its left
                        node.get<0>()->setLeft(&((*nodes)[nodeIndex]));
                    else
                        node.get<0>()->setRight(&((*nodes)[nodeIndex]));
                }
                // we increase node index
                nodeIndex++;
                if(!node.get<1>()->isLeaf()){
                    children.push(boost::make_tuple(node.get<1>(), node.get<1>()->right(), false) );
                    children.push(boost::make_tuple(node.get<1>(), node.get<1>()->left(), true) );
                }
            }
            // in the end we change root node
            _root = &((*nodes)[0]);
            _nodes = nodes;
            */
        }

		void setLeafPrimitives(Node* node, size_t primStartIdxs){
		    RW_ASSERT(0);
			if( !node->isLeaf() )
				RW_THROW("Not a leaf node!");

			node->primIdx() = _leafIndexes.size();
		}

		NodeIterator getRootIterator() const {
                return NodeIterator(_root, 0);
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
                    std::cout << "\t" << parent.second << "[label=\"" << parent.first->bv().calcVolume() <<"\"]\n";
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


		int getMaxTrisPerLeaf() const{return 1;};

	private:

		Node *_root;
		std::vector<size_t> _leafIndexes;
		//rw::common::Ptr<std::vector<Node> > _nodes;
	};


	typedef BinaryBVTree<rw::geometry::OBB<>, rw::geometry::Triangle<> > BinaryOBBPtrTreeD;
	typedef BinaryBVTree<rw::geometry::OBB<float>, rw::geometry::Triangle<float> > BinaryOBBPtrTreeF;


}
/*
    //! define traits of the NodeIterator
    template<class BV, class PRIM>
    struct Traits<proximity::BTPNode<BV,PRIM>::NodeIterator >{
        typedef BV BVType;
        typedef PRIM PRIMType;
        //typedef proximity::BTPNode<BV,PRIM> Node;
    };
*/
    //! define traits of the BinaryBVTree
    template<class BV, class PRIM> struct Traits<proximity::BinaryBVTree<BV,PRIM> >{
        typedef BV BVType;
        typedef PRIM PRIMType;
        typedef typename proximity::BTPNode<BV,PRIM> Node;
        typedef typename proximity::BTPNode<BV,PRIM>::NodeIterator NodeIterator;
    };



}

#endif /* BINARYTREE_HPP_ */
