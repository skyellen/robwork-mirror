#ifndef KDTREE_HPP_
#define KDTREE_HPP_

#include <boost/foreach.hpp>
#include <vector>
#include <list>
#include <algorithm>
#include <queue>
#include <rw/math/Q.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/common/macros.hpp>
#include <float.h>
#include <rw/math/Math.hpp>

namespace rwlibs {
namespace algorithms {

    // We can't define this as a static variable within the class, so we put it
    // here for now.
    const double kdtree_epsilon = 0.000000001;

template<class T>
class KDTree
{
private:

	struct TreeNode;

	size_t _dim;
	size_t _nrOfNodes;
	TreeNode *_root;
	std::vector<TreeNode>* _nodes;

public:
	//typedef std::pair<rw::math::Q,T> KDNode;
	typedef rw::math::Q KEY;
	struct KDNode {
		KDNode(rw::math::Q k, T val):key(k),value(val){}
		rw::math::Q key;
		T value;
	};

	struct KDResult {
		KDResult(KDNode *node, double d):n(node),dist(d){}
		KDNode *n;
		double dist;
	};

	/**
	 * @brief Constructor
	 * @param dim [in] the dimension of the keys in the KDTree
	 */
	KDTree(size_t dim);

	/**
	 * @brief destructor
	 */
	virtual ~KDTree(){};

	/**
	 * @brief Builds a KDTree from a list of key values and nodes. This method is more efficient
	 * than creating an empty KDTree and then inserting nodes
	 * @param nodes [in] a list of KDNode's
	 * @return if build succesfull then a pointer to a KD-tree is returned else NULL
	 */
	static KDTree<T>* buildTree(const std::list<KDNode*>& nodes){
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
	 * @return the nr of dimensions of this KD-Tree
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
	 * @param nnkey [in] the key that is to be found
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
	 * @param nnkey [in] the key to which the nearest neighbor is found
	 * @return the nearest neighbor to nnkey
	 */
	KDNode& nnSearch(const rw::math::Q& nnkey){
		//std::cout << "nnSearch" << std::endl;
		if (_root==NULL)
			RW_THROW("KDTree has no data!");

		rw::math::Q min(_dim), max(_dim);
		KDResult result(NULL, DBL_MAX);
		for(size_t i=0;i<_dim;i++){
			min(i) = -DBL_MAX;
			max(i) =  DBL_MAX;
		}
		nnSearchRec(nnkey, _root, min, max, result);
		if( result.n == NULL )
			RW_THROW("KDTree has no data!");
		return *result.n;
	}

	/**
	 * @brief finds all neighbors in the hyperelipse with radius radi and center in nnkey.
	 * @param nnkey [in] the center of the hyperelipse
	 * @param radi [in] the radius of the hyperelipse in euclidean 2-norm
	 * @param nodes [out] a container for all nodes that is found within the hyperelipse
	 */
	void nnSearchElipse(const rw::math::Q& nnkey,
						 const rw::math::Q& radi,
						 std::list<const KDNode*>& nodes)
	{
		//typedef std::pair<TreeNode*,size_t> QElem;
		using namespace rw::math;
		std::queue<TreeNode*> unhandled;
		unhandled.push( _root );
		double distSqr = MetricUtil::norm2Sqr(radi);
		double distRadi = sqrt(distSqr);
		Q low( _dim ),upp( _dim );
		for(size_t i=0;i<_dim;i++){
			low(i) = nnkey(i)-distRadi;
			upp(i) = nnkey(i)+distRadi;
		}

		nnSearchElipseRec(nnkey, _root, low, upp, distSqr, nodes);
	}

	/**
	 * @brief finds all neighbors in the hyperelipse with radius radi and center in nnkey.
	 * @param nnkey [in] the center of the hyperelipse
	 * @param radi [in] the radius of the hyperelipse in euclidean 2-norm
	 * @param nodes [out] a container for all nodes that is found within the hyperelipse
	 */
	void nnSearchElipseRect(const rw::math::Q& nnkey,
						 const rw::math::Q& radi,
						 std::list<const KDNode*>& nodes )
	{
		//typedef std::pair<TreeNode*,size_t> QElem;
		using namespace rw::math;
		std::queue<TreeNode*> unhandled;
		unhandled.push( _root );
		double distSqr = MetricUtil::norm2Sqr( radi );
		double distRadi = sqrt(distSqr);
		Q low( _dim ),upp( _dim );
		for(size_t i=0;i<_dim;i++){
			low(i) = nnkey(i)-distRadi;
			upp(i) = nnkey(i)+distRadi;
		}

		while( !unhandled.empty() ){
			//std::cout << "unhandled size: " << unhandled.size() << std::endl;
			TreeNode *n = unhandled.front();
			unhandled.pop();

			unsigned char axis = n->_axis;
			rw::math::Q &key = n->_kdnode->key;

			//std::cout << "Axis: " << axis << " depth: " << depth << std::endl;

			// if the key is in range then add it to the result
			size_t j;
			for( j=0; j<_dim && low[j]<=key[j] && upp[j] >= key[j]; j++ );
			//std::cout << j << "==" << _dim << " k:" << key << std::endl;
			if( j==_dim ) {// this is in range if
				double dist = MetricUtil::dist2Sqr(nnkey,key);
				//std::cout << "Dist: " << dist << " < " << distSqr << std::endl;
				if( dist<distSqr )
					nodes.push_back( n->_kdnode );
			}

			// add the children to the unhandled queue if the current dimension
			if( (low(axis) <= key(axis)) && (n->_left!=NULL) )
				unhandled.push( n->_left );
			if( (upp(axis) > key(axis)) && (n->_right!=NULL) )
				unhandled.push( n->_right );
		}
	}

	/**
	 * @brief finds all neighbors in the hyperrectangle defined by the lower bound and the
	 * upper bound
	 */
	void nnSearchRect(const rw::math::Q& low, const rw::math::Q& upp,
				 	  std::list<const KDNode*>& nodes ){
		//typedef std::pair<TreeNode*,size_t> QElem;
		std::queue<TreeNode*> unhandled;
		unhandled.push( _root );

		//std::cout << "nnSearchRect: "<< std::endl;
		//std::cout << "- low bound: "<< low << std::endl;
		//std::cout << "- upp bound: "<< upp << std::endl;

		while( !unhandled.empty() ){
			//std::cout << "unhandled size: " << unhandled.size() << std::endl;
			TreeNode *n = unhandled.front();
			unhandled.pop();

			unsigned char axis = n->_axis;
			rw::math::Q &key = n->_kdnode->key;

			//std::cout << "Axis: " << axis << " depth: " << depth << std::endl;

			// if the key is in range then add it to the result
			size_t j;
			for( j=0; j<_dim && low[j]<=key[j] && key[j] <= upp[j]; j++ );
			//std::cout << j << "==" << _dim << " k:" << key << std::endl;
			if( j==_dim ) // this is in range
				nodes.push_back( n->_kdnode );

			// add the children to the unhandled queue if the current dimension
			if( (low(axis) <= key(axis)) && (n->_left!=NULL) )
				unhandled.push( n->_left );
			if( (upp(axis) > key(axis)) && (n->_right!=NULL) )
				unhandled.push( n->_right );
		}
	};

private:

	KDTree(){};

	KDTree(TreeNode &root, std::vector<TreeNode> *nodes):
		_dim(root._kdnode->key.size()),
		_root(&root),_nodes(nodes)
	{
	};

	/**
	 * @brief Internal representation of a KD Tree Node. To save processing time when deleting
	 * TreeNodes, a boolean is kept that say if the node is deleted or not. If deleted all
	 * rutines kan skip the node and forward the call to its children.
	 */
	struct TreeNode {
	public:
		TreeNode():
			_left(NULL),_right(NULL),_kdnode(NULL),_deleted(false),_axis(0){};
		TreeNode(KDNode* node):
			_left(NULL),_right(NULL),_kdnode(node),_deleted(false),_axis(0){};
		TreeNode(TreeNode* left, TreeNode *right, KDNode* node):
			_left(left),_right(right),_kdnode(node),_deleted(false),_axis(0){};

		static void swap(TreeNode& n1, TreeNode& n2){
			std::swap<TreeNode*>(n1._left,n2._left);
			std::swap<TreeNode*>(n1._right,n2._right);
			std::swap<KDNode*>(n1._kdnode,n2._kdnode);
		}

		TreeNode *_left,*_right;
		KDNode* _kdnode;
		bool _deleted; //
		unsigned char _axis; // the splitting axis
	};

	struct SimpleCompare {
	private:
		size_t _dim;
	public:
		SimpleCompare(size_t dim):_dim(dim){};

		bool operator()(const TreeNode& e1, const TreeNode& e2) {
	        return e1._kdnode->key[_dim] < e2._kdnode->key[_dim] ;
	    }
	};


	static TreeNode* buildBalancedRec(std::vector<TreeNode>& tNodes, int startIdx,
							   int endIdx, size_t depth, size_t nrOfDims){
		if(endIdx<=startIdx)
			return NULL;

		//std::cout << "RecBuild(" << startIdx << "," << endIdx << ")" << std::endl;
		size_t len = endIdx-startIdx;

		size_t dim = depth % nrOfDims;
		// the compare func can
		std::sort( &tNodes[startIdx], &tNodes[endIdx-1], SimpleCompare(dim) );
		size_t medianIdx = startIdx+len/2;

		TreeNode &mNode = tNodes[medianIdx];
		mNode._axis = 0xFF&dim;
		mNode._left = BuildBalancedRec(tNodes, startIdx, medianIdx, depth+1, nrOfDims);
		mNode._right = BuildBalancedRec(tNodes, medianIdx+1, endIdx, depth+1, nrOfDims);
		return &mNode;
	}

	void nnSearchRec(const rw::math::Q& nnkey, TreeNode* node,
					   rw::math::Q &min, rw::math::Q &max,
					   KDResult& out){
		using namespace rw::math;
		if(node==NULL)
			return;
		size_t axis = node->_axis;
		//std::cout << "nnSearchRec("<< axis << ")" << std::endl;

		Q &key = node->_kdnode->key;
		double distSqr( MetricUtil::dist2Sqr(nnkey, key) );

		// if this node is closer than any other then update out
		if( distSqr<out.dist && !node->_deleted){
			out.dist = distSqr;
			out.n = node->_kdnode;
		}
		// stop if the distance is very small
		if( distSqr < kdtree_epsilon ) return;

		// call nnSearch recursively with closerNode,
		// closestNode and closestDistSqr is updated
		bool isLeftClosest = nnkey(axis)<key(axis);
		if( isLeftClosest ){
			// left is closest, backup split value and make the recursive call
			double maxTmp = max(axis);
			max(axis) = key(axis);
			nnSearchRec(nnkey, node->_left, min, max, out);
			// undo the change of max
			max(axis) = maxTmp;
		} else {
			// right is closest, backup split value and make the recursive call
			double minTmp = min(axis);
			min(axis) = key(axis);
			nnSearchRec(nnkey, node->_right, min, max, out);
			// undo the change of max
			min(axis) = minTmp;
		}

		// next check if fartherNode split plane lies closer than closestDistSqr
		if( Math::sqr(nnkey(axis)-key(axis)) >= out.dist )
			return;

		bool isLeftFarthest = !isLeftClosest;
		// if closest point in hyperrect of farther node is closer than closest
		// then call nnSearch recursively with farther node
		if( isLeftFarthest ){
			double maxTmp = max(axis);
			max(axis) = key(axis);
			rw::math::Q closest = Math::clampQ(nnkey, min, max);
			if( MetricUtil::dist2Sqr(nnkey, closest) < out.dist )
				nnSearchRec(nnkey, node->_left, min, max, out);
			// undo the change of max
			max(axis) = maxTmp;
		} else {
			double minTmp = min(axis);
			min(axis) = key(axis);
			rw::math::Q closest = Math::clampQ(nnkey, min, max);
			if( MetricUtil::dist2Sqr(nnkey, closest) < out.dist )
				nnSearchRec(nnkey, node->_right, min, max, out);
			// undo the change of max
			min(axis) = minTmp;
		}
	};

	void nnSearchElipseRec(const rw::math::Q& nnkey, TreeNode* node,
					   	   rw::math::Q &min, rw::math::Q &max,
					   	   double maxRadiSqr,
					   	   std::list<const KDNode*>& nodes){
		using namespace rw::math;
		if(node==NULL)
			return;
		size_t axis = node->_axis;
		//std::cout << "nnSearchRec("<< axis << ")" << std::endl;

		Q &key = node->_kdnode->key;
		double distSqr( MetricUtil::dist2Sqr(nnkey, key) );

		// if this node is closer than any other then update out
		if( distSqr<maxRadiSqr && !node->_deleted){
			nodes.push_back(node->_kdnode);
		}
		// stop if the distance is very small
		if( distSqr < kdtree_epsilon ) return;

		// call nnSearch recursively with closerNode,
		// closestNode and closestDistSqr is updated
		bool isLeftClosest = nnkey(axis)<key(axis);
		if( isLeftClosest ){
			// left is closest, backup split value and make the recursive call
			double maxTmp = max(axis);
			max(axis) = key(axis);
			nnSearchElipseRec(nnkey, node->_left, min, max,maxRadiSqr, nodes);
			// undo the change of max
			max(axis) = maxTmp;
		} else {
			// right is closest, backup split value and make the recursive call
			double minTmp = min(axis);
			min(axis) = key(axis);
			nnSearchElipseRec(nnkey, node->_right, min, max,maxRadiSqr, nodes);
			// undo the change of max
			min(axis) = minTmp;
		}

		// next check if fartherNode split plane lies closer than closestDistSqr
		if( Math::sqr(nnkey(axis)-key(axis)) >= maxRadiSqr )
			return;

		bool isLeftFarthest = !isLeftClosest;
		// if closest point in hyperrect of farther node is closer than closest
		// then call nnSearch recursively with farther node
		if( isLeftFarthest ){
			double maxTmp = max(axis);
			max(axis) = key(axis);
			rw::math::Q closest = Math::clampQ(nnkey, min, max);
			if( MetricUtil::dist2Sqr(nnkey, closest) < maxRadiSqr )
				nnSearchElipseRec(nnkey, node->_left, min, max,maxRadiSqr, nodes);
			// undo the change of max
			max(axis) = maxTmp;
		} else {
			double minTmp = min(axis);
			min(axis) = key(axis);
			rw::math::Q closest = Math::clampQ(nnkey, min, max);
			if( MetricUtil::dist2Sqr(nnkey, closest) < maxRadiSqr )
				nnSearchElipseRec(nnkey, node->_right, min, max,maxRadiSqr, nodes);
			// undo the change of max
			min(axis) = minTmp;
		}
	};
};

template<class T>
KDTree<T>::KDTree(size_t dim):
	_dim(dim),_nrOfNodes(0),_root(NULL),
	_nodes(new std::vector<TreeNode>())
{
}

} //namespace algorithms
} // namespace rwlibs

#endif /*KDTREE_HPP_*/
