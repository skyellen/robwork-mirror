#ifndef KDTREE_HPP_
#define KDTREE_HPP_

#include <boost/foreach.hpp>
#include <vector>
#include <list>
#include <algorithm>
#include <queue>
#include <rw/math/Q.hpp>


template<class T>
class KDTree
{
private:
	
	struct TreeNode;
	
	size_t _dim;
	size_t _nrOfNodes;
	std::vector<TreeNode>* _nodes;
	TreeNode *_root;
	
public:
	//typedef std::pair<rw::math::Q,T> KDNode;
	struct KDNode {
		rw::math::Q key;
		T value;
	};
	
	/**
	 * @brief Constructor
	 * @param dim [in] the dimension of the keys in the KDTree
	 */
	KDTree(size_t dim);
	
	/**
	 * @brief destructor
	 */
	virtual ~KDTree();
	
	/**
	 * @brief Builds a KDTree from a list of key values and nodes. This method is more efficient 
	 * than creating an empty KDTree and then inserting nodes
	 * @param 
	 */
	static KDTree<T>* BuildTree(const std::list<KDNode*>& nodes){
		if(nodes.size()==0)
			return NULL;
		
		// create all tree nodes in a list
		std::vector<TreeNode> *tNodes = new std::vector<TreeNode>( nodes.size() );
		
		// copy the KDNodes into the tree nodes
		int i=0;
		BOOST_FOREACH(KDNode* n, nodes){
			(*tNodes)[i]._kdnode = n;
			i++;
		}
		
		// create a simple median balanced tree
		size_t nrOfDims = nodes.front()->key.size(); 
		TreeNode *root = BuildBalancedRec(*tNodes, 0, tNodes->size(), 0, nrOfDims);
		
		return new KDTree<T>(*root, tNodes);
	}

	/**
	 * @brief gets the number of dimensions that this KDTree supports
	 */
	size_t getDimensions() const{
		return _dim;
	};
	
	/**
	 * @brief adds a key value pair to the KDTree.
	 * @param key [in] must be the same length as the dimensionality of the KDTree
	 * @param val [in] value that is to be stored at the keys position
	 */
	void addNode(const rw::math::Q& key, T val);
	
	/**
	 * @brief finds the KDNode with key equal to nnkey
	 * @return KDNode with key equal to nnkey if existing, else NULL
	 */
	KDNode* search(const rw::math::Q& nnkey){
		TreeNode *tmpNode = _root;
		for(size_t lev=0; tmpNode!=NULL; lev = (lev+1)%_dim ){
			rw::math::Q& key = tmpNode->_kdnode->key;
			if( nnkey(lev)==key(lev) && !(tmpNode->_deleted) && (nnkey == key) ) {
				return tmpNode->_kdnode;
			} else if( nnkey(lev) > key(lev) ){
				tmpNode = tmpNode->_right;
			} else {
				tmpNode = tmpNode->_left;
			}
		}
		return NULL;
	};
	
	/**
	 * @brief finds the KDNode with the key closest too nnkey
	 */
	KDNode& searchNeighbor(const rw::math::Q& nnkey){
		
	}
	
	/**
	 * @brief finds all neighbors in the hyperelipse with radius radi and center in nnkey.
	 */
	void searchNeighbors(const rw::math::Q& nnkey, 
						 const rw::math::Q& radi, 
						 std::list<const KDNode*>& nodes ); 
	
	/**
	 * @brief finds all neighbors in the hyperrectangle defined by the lower bound and the
	 * upper bound 
	 */
	void searchNeighbors(std::pair<rw::math::Q, rw::math::Q > bound, 
						  std::list<const KDNode*>& nodes ){
		typedef std::pair<TreeNode*,size_t> QElem;
		rw::math::Q &low = bound.first;
		rw::math::Q &upp = bound.second;
		std::queue<QElem > unhandled;
		unhandled.push(QElem(_root,0));
		
		while( !unhandled.empty() ){
			QElem tmp = unhandled.front();
			TreeNode *n = tmp.first;
			size_t depth = tmp.second;
			size_t axis = depth % _dim;
			unhandled.pop();
			rw::math::Q &key = n->_kdnode->key;

			// if the key is in range then add it to the result
			size_t j;
			for( j=0; j<_dim && low[j]<=key[j] && upp[j] >= key[j]; j++ );
			if(j==_dim) // this is in range
				nodes.push_back( n->_kdnode );
			
			// add the children to the unhandled queue if the current dimension
			if( (low(axis) <= key(axis)) && (n->_left!=NULL) )
				unhandled.push( QElem( n->_left, (depth+1)%_dim ) );
			if( (upp(axis) > key(axis)) && (n->_right!=NULL) )
				unhandled.push( QElem( n->_right, (depth+1)%_dim ) );
		}
	};
	
private:
	
	KDTree(){};
	
	KDTree(TreeNode &root, std::vector<TreeNode> *nodes){
		
	};
	
	//vc static BuildRec()
	
	/**
	 * @brief Internal representation of a KD Tree Node. To save processing time when deleting
	 * TreeNodes, a boolean is kept that say if the node is deleted or not. If deleted all
	 * rutines kan skip the node and forward the call to its children.
	 */
	struct TreeNode {
	public:
		TreeNode():
			_left(NULL),_right(NULL),_kdnode(NULL),_deleted(false){};
		TreeNode(KDNode* node):
			_left(NULL),_right(NULL),_kdnode(node),_deleted(false){};
		TreeNode(TreeNode* left, TreeNode *right, KDNode* node):
			_left(left),_right(right),_kdnode(node),_deleted(false){};
		
		static void Swap(TreeNode& n1, TreeNode& n2){
			std::swap<TreeNode*>(n1._left,n2._left);
			std::swap<TreeNode*>(n1._right,n2._right);
			std::swap<KDNode*>(n1._kdnode,n2._kdnode);
		}
		
		TreeNode *_left,*_right;
		KDNode* _kdnode;
		bool _deleted; // 
	};

	struct HyperRect {
		HyperRect(rw::math::Q min, rw::math::Q max):
			_min(min),_max(max) {};
		
		rw::math::Q getCenter(rw::math::Q &res){
			return _min+(_max-_min)/2.0;
		}
		
		double getCenter(size_t dim){
			return _min(dim)+(_max(dim)-_min(dim))/2.0;
		}
		
		rw::math::Q _min;
		rw::math::Q _max;
	};
	
	struct SimpleCompare {
	private:
		size_t _dim;
	public:
		SimpleCompare(size_t dim):_dim(dim){};

		bool operator()(const TreeNode& e1, const TreeNode& e2) {         
	        return e1._kdnode->key[_dim] > e2._kdnode->key[_dim] ;
	    }
	};

	
	static TreeNode* BuildBalancedRec(std::vector<TreeNode>& tNodes, size_t startIdx,
							   size_t endIdx, size_t depth, size_t nrOfDims){
		size_t len = endIdx-startIdx;
		if(len==0)
			return NULL;
		
		size_t dim = depth % nrOfDims;
		// the compare func can 
		std::sort( &tNodes[startIdx], &tNodes[endIdx], SimpleCompare(dim) );
		size_t medianIdx = startIdx+len/2;
		
		TreeNode &mNode = tNodes[medianIdx];
		mNode._left = BuildBalancedRec(tNodes, startIdx, medianIdx-1, depth+1, nrOfDims);
		mNode._right = BuildBalancedRec(tNodes, medianIdx+1, endIdx, depth+1, nrOfDims);
		return &mNode;
	}
};

template<class T>
KDTree<T>::KDTree(size_t dim):
	_dim(dim),_nrOfNodes(0)
{	
}


#endif /*KDTREE_HPP_*/
