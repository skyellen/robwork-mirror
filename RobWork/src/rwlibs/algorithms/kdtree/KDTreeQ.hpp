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


#ifndef RWLIBS_ALGORTIHMS_KDTREEQ_HPP_
#define RWLIBS_ALGORTIHMS_KDTREEQ_HPP_

#include <boost/foreach.hpp>
#include <vector>
#include <list>
#include <algorithm>
#include <queue>
#include <map>
#include <rw/math/Q.hpp>
#include <rw/math/MetricUtil.hpp>
#include "KDTree.hpp"
#include <rw/common/macros.hpp>
#include <float.h>
#include <rw/math/Math.hpp>

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

#include <boost/tuple/tuple.hpp>

namespace rwlibs { namespace algorithms {

    /** \addtogroup algorithms */
    /*@{*/

    /**
     * @brief a space partitioning structure for organizing points in k-dimensional space.
     * Used for searches involving multi.dimensional search keys, including nearest
     * neighbor and range search.
     *
     * This KDTree implementation takes any value type but the key is constrained to a
     * rw::math::Q
     */
    template<class VALUE_TYPE>
    class KDTreeQ: public rw::common::Serializable
    {
    private:

        struct TreeNode;

    public:
        typedef rw::common::Ptr<KDTreeQ<VALUE_TYPE> > Ptr;

        typedef rw::math::Q KEY;

        //! a struct for the node in the tree
        struct KDNode {
            KDNode(rw::math::Q k, VALUE_TYPE val):key(k),value(val){}
            rw::math::Q key;
            VALUE_TYPE value;

            //template<class T>
            //T valueAs() { return boost::any_cast<T>(value); }
            //template<class T>
            //T valueAs() const { return boost::any_cast<T>(value); }
        };

        struct KDResult {
            KDResult(KDNode *node, double d):n(node),dist(d){}
            KDNode *n;
            double dist;
        };

        typedef KDNode Node;
        typedef KDResult Result;
        typedef VALUE_TYPE Value;

        /**
        * @brief Constructor
        * @param dim [in] the dimension of the keys in the KDTreeQ
        */
        KDTreeQ(size_t dim):
            _dim(dim),
            _nrOfNodes(0),
            _root(NULL),
            _nodes(new std::vector<TreeNode*>())
        {
        }


        /**
        * @brief destructor
        */
        virtual ~KDTreeQ(){
            delete _root;
            delete _nodes;
        };


        /**
        * @brief Builds a KDTreeQ from a list of key values and nodes. This method is more efficient
        * than creating an empty KDTreeQ and then inserting nodes
        * @param nodes [in] a list of KDNode's
        * @return if build succesfull then a pointer to a KD-tree is returned else NULL
        */
        static KDTreeQ* buildTree(std::vector<KDNode>& nodes);

        /**
        * @brief Builds a KDTreeQ from a list of key values and nodes. This method is more efficient
        * than creating an empty KDTreeQ and then inserting nodes
        * @param nodes [in] a list of KDNode's
        * @return if build succesfull then a pointer to a KD-tree is returned else NULL
        */
        static KDTreeQ* buildTree(const std::vector<KDNode*>& nodes);

        /**
        * @brief gets the number of dimensions that this KDTreeQ supports
        * @return the nr of dimensions of this KD-Tree
        */
        size_t getDimensions() const { return _dim; };

        /**
        * @brief adds a key value pair to the KDTreeQ.
        * @param key [in] must be the same length as the dimensionality of the KDTreeQ
        * @param val [in] value that is to be stored at the keys position
        */
        void addNode(const rw::math::Q& key, VALUE_TYPE val);

        /**
         * @brief remove the node with key nnkey
         * @param nnkey [in] the key of the node to remove
         * @return
         */
        void removeNode(const rw::math::Q& nnkey);

        /**
        * @brief finds the KDNode with key equal to nnkey
        * @param nnkey [in] the key that is to be found
        * @return KDNode with key equal to nnkey if existing, else NULL
        */
        KDNode* search(const rw::math::Q& nnkey);

        /**
        * @brief finds the KDNode with the key closest too nnkey
        * @param nnkey [in] the key to which the nearest neighbor is found
        * @return the nearest neighbor to nnkey
        */
        KDNode& nnSearch(const rw::math::Q& nnkey);


        /**
        * @brief finds all neighbors in the hyperelipse with radius radi and center in nnkey.
        * @param nnkey [in] the center of the hyperelipse
        * @param radi [in] the radius of the hyperelipse in euclidean 2-norm
        * @param nodes [out] a container for all nodes that is found within the hyperelipse
        */
        void nnSearchElipse(const rw::math::Q& nnkey,
                            const rw::math::Q& radi,
                            std::list<const KDNode*>& nodes);

        /**
        * @brief finds all neighbors in the hyperelipse with radius radi and center in nnkey.
        * @param nnkey [in] the center of the hyperelipse
        * @param radi [in] the radius of the hyperelipse in euclidean 2-norm
        * @param nodes [out] a container for all nodes that is found within the hyperelipse
        */
        void nnSearchElipseRect(const rw::math::Q& nnkey,
                                const rw::math::Q& radi,
                                std::list<const KDNode*>& nodes );

        /**
        * @brief finds all neighbors in the hyperrectangle defined by the lower bound and the
        * upper bound
        */
        void nnSearchRect(const rw::math::Q& low, const rw::math::Q& upp,
                          std::list<const KDNode*>& nodes );

    private:

        size_t _dim;
        size_t _nrOfNodes;
        TreeNode *_root;
        std::vector<TreeNode*>* _nodes;

    public:

        void read(rw::common::InputArchive& iarchive, const std::string& id){
            std::string name, data;
            int dim, nrNodes;
            boost::uint64_t rootId;
            iarchive.read(dim, "dim");
            iarchive.read(nrNodes, "nrNodes");
            rootId = iarchive.readUInt64("rootId");

            std::vector<TreeNode*>* nodes = new std::vector<TreeNode*>(nrNodes);
            std::vector<int> idToNodeIdx(nrNodes);
            std::map<boost::uint64_t, boost::tuple<TreeNode*,boost::uint64_t,boost::uint64_t> > toNode;
            for(int i=0;i<nrNodes;i++){
                (*nodes)[i] = new TreeNode();
                TreeNode &node = *(*nodes)[i];

                boost::uint64_t id = iarchive.readUInt64("id");
                node._axis = iarchive.readInt("axis");
                node._deleted = iarchive.readBool("del");

                boost::uint64_t leftId = iarchive.readUInt64("left");
                boost::uint64_t rightId = iarchive.readUInt64("right");

                iarchive.read(node._kdnode->key, "Q");
                iarchive.read(node._kdnode->value, "value");

                toNode[id] = boost::make_tuple(&node,leftId,rightId);
                idToNodeIdx[i] = id;
            }
            toNode[0] = boost::make_tuple((TreeNode*)NULL,(boost::uint64_t)0,(boost::uint64_t)0);

            // finally travel through nodes and set the correct left/right values
            for(int i=0;i<nrNodes;i++){
                TreeNode &node = *(*nodes)[i];

                boost::tuple<TreeNode*,boost::uint64_t,boost::uint64_t> val = toNode[ idToNodeIdx[i] ];
                int leftIdx = boost::get<1>(val);
                node._left = boost::get<0>( toNode[leftIdx] );
                node._right = boost::get<0>( toNode[boost::get<2>(val)]);
            }
            TreeNode *root = boost::get<0>(toNode[rootId]);
            _dim = root->_kdnode->key.size();
            _root = root;
            _nodes = nodes;
        }

        void write(rw::common::OutputArchive& oarchive, const std::string& id) const {
            oarchive.write(_dim, "dim");
            oarchive.write((int)_nodes->size(), "nrNodes");
            oarchive.write((boost::uint64_t)_root, "rootId");
            RW_ASSERT(_nrOfNodes==_nodes->size());
            for(std::size_t i=0;i<_nodes->size();i++){
                const TreeNode &node = *(*_nodes)[i];
                oarchive.write((boost::uint64_t)&node, "id");
                oarchive.write((int)node._axis, "axis");
                oarchive.write(node._deleted, "del");
                oarchive.write((boost::uint64_t)node._left, "left");
                oarchive.write((boost::uint64_t)node._right, "right");

                oarchive.write( node._kdnode->key, "Q");
                oarchive.write( node._kdnode->value, "value");
            }
        }

/*
        static void save(RWOutputArchive& archive);
        static KDTreeQ* load(RWInputArchive& archive){

        }

        static void save(KDTreeQ* tree, ValueSerializer& serializer, const std::string& filename);
        static KDTreeQ* load(ValueSerializer& serializer, const std::string& filename);
        */



    private:
        //! default constructor
        KDTreeQ(){};

        //! constructor
        KDTreeQ(TreeNode *root, std::vector<TreeNode*> *nodes):
            _dim(root->_kdnode->key.size()),
            _root(root),_nodes(nodes)
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
                _left(NULL),_right(NULL),_kdnode(NULL),_deleted(false),_axis(0)
            {};

            TreeNode(KDNode* node):
                _left(NULL),_right(NULL),_kdnode(node),_deleted(false),_axis(0)
            {
            };

            TreeNode(TreeNode* left, TreeNode *right, KDNode* node):
                _left(left),_right(right),_kdnode(node),_deleted(false),_axis(0)
            {};

            static void swap(TreeNode& n1, TreeNode& n2){
                std::swap(n1._left,n2._left);
                std::swap(n1._right,n2._right);
                std::swap(n1._kdnode,n2._kdnode);
            }

            TreeNode *_left,*_right;
            KDNode* _kdnode;
            bool _deleted; //
            unsigned char _axis; // the splitting axis
        };
/*
        struct SimpleCompare {
        private:
            size_t _dim;
        public:
            SimpleCompare(size_t dim):_dim(dim){};

            bool operator()(const TreeNode& e1, const TreeNode& e2) {
                RW_ASSERT(e1._kdnode);
                RW_ASSERT(e2._kdnode);
                return e1._kdnode->key[_dim] < e2._kdnode->key[_dim] ;
            }
        };
*/
        struct SimpleCompare2 {
        private:
            size_t _dim;
        public:
            SimpleCompare2(size_t dim):_dim(dim){};

            bool operator()(const TreeNode* e1, const TreeNode* e2) {
                return e1->_kdnode->key[_dim] < e2->_kdnode->key[_dim] ;
            }
        };

        static TreeNode* buildBalancedRec(std::vector<TreeNode*>& tNodes, int startIdx,
                                          int endIdx, size_t depth, size_t nrOfDims){
            if(endIdx<=startIdx)
                return NULL;

            //std::cout << "RecBuild(" << startIdx << "," << endIdx << ")" << std::endl;
            size_t len = endIdx-startIdx;
            size_t dim = depth % nrOfDims;

            // the compare func can
            std::sort( tNodes.begin()+startIdx, tNodes.begin()+endIdx, SimpleCompare2(dim) );
            // check sorting is okay
            for(int i=startIdx;i<endIdx-1;i++)
                if( tNodes[i]->_kdnode->key[dim] > tNodes[i+1]->_kdnode->key[dim] )
                    RW_WARN(" sort not working!" << i);
            size_t medianIdx = startIdx+len/2;
            TreeNode *mNode = tNodes[medianIdx];
            mNode->_axis = 0xFF&dim;
            mNode->_left = buildBalancedRec(tNodes, startIdx, (int)medianIdx, depth+1, nrOfDims);
            mNode->_right = buildBalancedRec(tNodes, (int)(medianIdx+1), endIdx, depth+1, nrOfDims);
            return mNode;
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
            //std::cout << "le" << std::endl;

            // if this node is closer than any other then update out
            if( distSqr<out.dist && !node->_deleted){
                out.dist = distSqr;
                out.n = node->_kdnode;
            }
            // stop if the distance is very small
            if( distSqr < kdtree_epsilon ) return;
            //std::cout << "1" << std::endl;
            // call nnSearch recursively with closerNode,
            // closestNode and closestDistSqr is updated
            bool isLeftClosest = nnkey(axis)<key(axis);
            if( isLeftClosest ){
                //std::cout << "left" << std::endl;
                // left is closest, backup split value and make the recursive call
                double maxTmp = max(axis);
                max(axis) = key(axis);
                nnSearchRec(nnkey, node->_left, min, max, out);
                // undo the change of max
                max(axis) = maxTmp;
            } else {
                //std::cout << "right" << std::endl;
                // right is closest, backup split value and make the recursive call
                double minTmp = min(axis);
                min(axis) = key(axis);
                nnSearchRec(nnkey, node->_right, min, max, out);
                // undo the change of max
                min(axis) = minTmp;
            }
            //std::cout << "2" << std::endl;

            // next check if fartherNode split plane lies closer than closestDistSqr
            if( Math::sqr(nnkey(axis)-key(axis)) >= out.dist )
                return;
            //std::cout << "3" << std::endl;

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
    KDTreeQ<T>* KDTreeQ<T>::buildTree(std::vector<typename KDTreeQ<T>::KDNode>& nodes){
        if(nodes.size()==0)
            return NULL;

        // create all tree nodes in a list
        std::vector<TreeNode*> *tNodes = new std::vector<TreeNode*>( nodes.size() );
        // copy the KDNodes into the tree nodes
        for(unsigned int i=0;i<tNodes->size();i++){
            (*tNodes)[i] = new TreeNode();
            (*tNodes)[i]->_kdnode = new KDTreeQ<T>::KDNode( nodes[i] );
        }

        // create a simple median balanced tree
        size_t nrOfDims = nodes.front().key.size();
        TreeNode *root = buildBalancedRec(*tNodes, 0, (int)tNodes->size(), 0, nrOfDims);
        return new KDTreeQ<T>(root, tNodes);
    }

    template<class T>
    KDTreeQ<T>* KDTreeQ<T>::buildTree(const std::vector<typename KDTreeQ<T>::KDNode*>& nodes){
        if(nodes.size()==0)
            return NULL;

        // create all tree nodes in a list
        std::vector<TreeNode*> *tNodes = new std::vector<TreeNode*>( nodes.size() );

        // copy the KDNodes into the tree nodes
        int i=0;
        BOOST_FOREACH(KDNode* n, nodes){
            (*tNodes)[i] = new TreeNode();
            (*tNodes)[i]->_kdnode = n;
            i++;
        }

        // create a simple median balanced tree
        size_t nrOfDims = nodes.front()->key.size();
        TreeNode *root = buildBalancedRec(*tNodes, 0, tNodes->size(), 0, nrOfDims);

        return new KDTreeQ<T>(root, tNodes);
    }

    template<class T>
    typename KDTreeQ<T>::KDNode* KDTreeQ<T>::search(const rw::math::Q& nnkey){
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
    }

    template<class T>
    typename KDTreeQ<T>::KDNode& KDTreeQ<T>::nnSearch(const rw::math::Q& nnkey){
        //std::cout << "nnSearch " << _dim << std::endl;

        RW_ASSERT(nnkey.size()==_dim);
        if (_root==NULL)
            RW_THROW("KDTreeQ has no data!");

        rw::math::Q min(_dim), max(_dim);
        KDResult result(NULL, DBL_MAX);
        for(size_t i=0;i<_dim;i++){

            min(i) = -DBL_MAX;
            max(i) =  DBL_MAX;
        }
        //std::cout << "nnSearchRec" << std::endl;
        nnSearchRec(nnkey, _root, min, max, result);
        if( result.n == NULL )
            RW_THROW("KDTreeQ has no data!");
        return *result.n;
    }

    template<class T>
    void KDTreeQ<T>::removeNode(const rw::math::Q& nnkey){
        TreeNode *tmpNode = _root;
          for(size_t lev=0; tmpNode!=NULL; lev = (lev+1)%_dim ){
              rw::math::Q& key = tmpNode->_kdnode->key;
              if( nnkey(lev)==key(lev) && !(tmpNode->_deleted) && (nnkey == key) ) {
                  tmpNode->_deleted = true;
                  return;
              } else if( nnkey(lev) > key(lev) ){
                  tmpNode = tmpNode->_right;
              } else {
                  tmpNode = tmpNode->_left;
              }
          }
    }

    template<class T>
    void KDTreeQ<T>::nnSearchElipse(const rw::math::Q& nnkey,
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

    template<class T>
    void KDTreeQ<T>::nnSearchElipseRect(const rw::math::Q& nnkey,
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

            //std::cout << "Axis: " << axis << std::endl;

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

    template<class T>
    void KDTreeQ<T>::nnSearchRect(const rw::math::Q& low, const rw::math::Q& upp,
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

            //std::cout << "Axis: " << axis << std::endl;

            // if the key   is in range then add it to the result
            size_t j;
            for( j=0; j<_dim && low[j]<=key[j] && key[j] <= upp[j]; j++ );

            //std::cout << j << "==" << _dim << " k:" << key << std::endl;
            if( j==_dim && n->_deleted==false) // this is in range
                nodes.push_back( n->_kdnode );

            // add the children to the unhandled queue if the current dimension
            if( (low(axis) <= key(axis)) && (n->_left!=NULL) )
                unhandled.push( n->_left );

            if( (upp(axis) > key(axis)) && (n->_right!=NULL) )
                unhandled.push( n->_right );
        }
    }

    template<class T>
    void KDTreeQ<T>::addNode(const rw::math::Q& nnkey, T val){
        // first test if root is empty. if it is add value as root
        if(_root==NULL){
            _nodes->push_back( new TreeNode(new KDNode(nnkey,val)) );
            _root =  _nodes->back();
            return;
        }

        // else find the parent in which this is to be inserted
        TreeNode *tmpNode = _root;
        // find the leaf in which to insert the new value
        for(size_t lev=0; tmpNode!=NULL; lev = (lev+1)%_dim ){
            KEY& key = tmpNode->_kdnode->key;

            if( nnkey(lev) > key(lev) ){
                if(tmpNode->_right==NULL){
                    TreeNode *node = new TreeNode(new KDNode(nnkey,val));
                    node->_axis = (lev+1)%_dim;
                    _nodes->push_back(node);
                    tmpNode->_right =  _nodes->back();
                    return;
                }
                tmpNode = tmpNode->_right;
            } else {
                if(tmpNode->_left==NULL){
                    TreeNode *node = new TreeNode(new KDNode(nnkey,val));
                    node->_axis = (lev+1)%_dim;
                    _nodes->push_back(node);
                    tmpNode->_left =  _nodes->back();
                    return;
                }
                tmpNode = tmpNode->_left;
            }
        }
    }





    /**@}*/

} //namespace algorithms
} // namespace rwlibs

#endif
