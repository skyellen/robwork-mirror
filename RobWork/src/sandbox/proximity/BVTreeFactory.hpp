#ifndef BVTREEBUILDER_HPP_
#define BVTREEBUILDER_HPP_


#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/TriMesh.hpp>


#include <rw/common/macros.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Math.hpp>

#include <rw/common/Timer.hpp>

#include <vector>
#include <stack>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/foreach.hpp>


#include <sandbox/geometry/OBB.hpp>
#include <sandbox/geometry/IndexedArray.hpp>
#include <sandbox/geometry/IndexedTriArray.hpp>

#include "BinaryBVTree.hpp"
#include "BinaryIdxBVTree.hpp"

namespace rw {
namespace proximity {

	/**
	 * @brief factory for creating bounding volume trees.
	 *
	 */
	class BVTreeFactory {
	public:

		/**
		 * @brief interface of bounding volume splitting strategy
		 */
		template<class BV>
		struct BVSplitterStrategy {
			virtual size_t partitionMesh(rw::geometry::IndexedTriArray<>& mesh, BV bv) = 0;
		};

		/**
		 * @brief interface of bounding volume factory
		 */
		template<class BV>
		struct BVFactory {
			//! @brief create a BV
			virtual BV makeBV(const rw::geometry::TriMesh& mesh) = 0;
		};

		/**
		 * @brief creates a splitter strategy for OBB tree construction using
		 * an object median splitting strategy.
		 *
		 * splits a trimesh in one of the axis of a bounding volume. The splitter use
		 * a median based strategy where the splitpoint is determined as the object median
		 * and as such is suitable for creating balanced OBB trees.
		 * Median is found in O(n log n) time
		 */
		rw::common::Ptr<BVSplitterStrategy<rw::geometry::OBB<> > > makeOBBMedianSplitter();

		/**
		 * @brief creates a splitter strategy for OBB tree construction using
		 * a spatial median splitting strategy.
		 *
		 * splits a trimesh in one of the axis of a bounding volume such that the bounding
		 * volume is split in two equal parts. Median is found in constant time.
		 */
		rw::common::Ptr<BVSplitterStrategy<rw::geometry::OBB<> > > makeOBBSpatialMedianSplitter();

		/**
		 * @brief creates a splitter strategy for obb tree construction using mean
		 * splitting strategy.
		 *
		 * The strategy splits a trimesh in one of the axes of the OBB. The splitting
		 * point is determined as the mean on one of the OBB axes. Performs in O(n)
		 * @return Splitter strategy for OBB tree construction
		 */
		rw::common::Ptr<BVSplitterStrategy<rw::geometry::OBB<> > > makeOBBMeanSplitter();


		/**
		 * @brief creates an OBB factory that fits obb's to triangle meshes
		 * by calculating the covariance of the triangle mesh and use the eigen vectors
		 * of the covariance as the axes of the OBB.
		 * @return OBB factory
		 */
		rw::common::Ptr<BVFactory<rw::geometry::OBB<> > > makeOBBCovarFactory();

		/**
		 * @brief creates an OBB tree using a covariance OBB factory and a object median splitting strategy.
		 * @param mesh [in] the mesh that should be decomposed into a OBB tree
		 * @param maxTrisInLeaf [in] the maximum number of tris that are allowed in each leaf node
		 * @return OBB tree
		 */
		BinaryBVTree<rw::geometry::OBB<> >* makeTopDownOBBTreeCovarMedian(rw::geometry::TriMesh::Ptr mesh, int maxTrisInLeaf=1){
			rw::common::Ptr<BVFactory<rw::geometry::OBB<> > > bvfactory = makeOBBCovarFactory();
			rw::common::Ptr<BVSplitterStrategy<rw::geometry::OBB<> > > splitter = makeOBBMedianSplitter();
			return makeTopDownTree<BinaryBVTree<rw::geometry::OBB<> > >(mesh, *bvfactory, *splitter, maxTrisInLeaf);
		}

        BinaryBVTree<rw::geometry::OBB<> >* makeTopDownOBBTreeCovarSpatialMedian(rw::geometry::TriMesh::Ptr mesh, int maxTrisInLeaf=1){
            rw::common::Ptr<BVFactory<rw::geometry::OBB<> > > bvfactory = makeOBBCovarFactory();
            rw::common::Ptr<BVSplitterStrategy<rw::geometry::OBB<> > > splitter = makeOBBSpatialMedianSplitter();
            return makeTopDownTree<BinaryBVTree<rw::geometry::OBB<> > >(mesh, *bvfactory, *splitter, maxTrisInLeaf);
        }

		BinaryBVTree<rw::geometry::OBB<> >* makeTopDownOBBTreeCovarMean(rw::geometry::TriMesh::Ptr mesh, int maxTrisInLeaf=1){
			rw::common::Ptr<BVFactory<rw::geometry::OBB<> > > bvfactory = makeOBBCovarFactory();
			rw::common::Ptr<BVSplitterStrategy<rw::geometry::OBB<> > > splitter = makeOBBMeanSplitter();
			return makeTopDownTree<BinaryBVTree<rw::geometry::OBB<> > >(mesh, *bvfactory, *splitter, maxTrisInLeaf);
		}

        BinaryIdxBVTree<rw::geometry::OBB<> >* makeTopDownOBBIdxTreeCovarMedian(rw::geometry::TriMesh::Ptr mesh, int maxTrisInLeaf=1){
            rw::common::Ptr<BVFactory<rw::geometry::OBB<> > > bvfactory = makeOBBCovarFactory();
            rw::common::Ptr<BVSplitterStrategy<rw::geometry::OBB<> > > splitter = makeOBBMedianSplitter();
            return makeTopDownTree<BinaryIdxBVTree<rw::geometry::OBB<> > >(mesh, *bvfactory, *splitter, maxTrisInLeaf);
        }


		/**
		 * @brief general function for constructing a binary bounding volume tree in a top down fashion.
		 *
		 * @param mesh [in] the mesh on which to construct the bounding volume tree
		 * @param bvFactory [in] a factory for creating/fitting bounding volumes given a triangle mesh
		 * @param splitter [in] divides a mesh into 2 meshes by sorting the mesh and providing a splitting index
		 * @param maxTrisInLeaf [in] the maximum number of tris that are allowed in each leaf node
		 * @return
		 */
		template<class BINARYBVTREE>
		static BINARYBVTREE* makeTopDownTree(rw::geometry::TriMesh::Ptr mesh,
							BVFactory<typename BINARYBVTREE::BVType>& bvFactory,
							BVSplitterStrategy<typename BINARYBVTREE::BVType>& splitter,
							int maxTrisInLeaf=1){
			using namespace rw::math;
			using namespace rw::geometry;

			// we create the binary tree
			BINARYBVTREE* tree = new BINARYBVTREE(mesh);
			RW_WARN("4");
			// create a proxy for the triangle mesh
			IndexedTriArray<> idxArray(mesh);
			RW_WARN("4");
			// now for each tri soup indicated by the triangle indexes compute a OBB sub tree
			typename BINARYBVTREE::node_iterator root = tree->createRoot();
			recursiveTopDownTree<BINARYBVTREE>(tree, root, idxArray, bvFactory, splitter, maxTrisInLeaf);
			RW_WARN("4");
			//std::cout << "IDX MAP ARRAY" << std::endl;
			//BOOST_FOREACH(int idx, idxArray.getIndexes()){
			//    std::cout << idx << "\n";
			//}

			//std::cout << "tree prims: " << (*root)->nrOfPrims() << std::endl;

			return tree;
		}

		/**
		 * @brief recursive top down construction of a bounding volume tree
		 * @param tree
		 * @param node
		 * @param mesh
		 * @param bvFactory
		 * @param splitter
		 * @param maxTrisInLeaf
		 */
		template<class BINARYBVTREE>
		static void recursiveTopDownTree(BINARYBVTREE* tree,
									typename BINARYBVTREE::node_iterator &node,
									rw::geometry::IndexedTriArray<> mesh,
									BVFactory<typename BINARYBVTREE::BVType>& bvFactory,
									BVSplitterStrategy<typename BINARYBVTREE::BVType> &splitter,
									size_t maxTrisInLeaf){
		    typedef typename BINARYBVTREE::BVType BV;
		    typedef BINARYBVTREE BVTree;

			if(mesh.getSize()==0){
			    RW_ASSERT(0); // we should not arrive at this.
			} else if(mesh.getSize()<=maxTrisInLeaf){
				// make a leaf node
			    BV bv = bvFactory.makeBV( mesh );
			    tree->setPrimIdx( mesh.getGlobalIndex(0), node);
			    tree->setNrOfPrims( mesh.getSize(), node);
			    tree->setBV(bv, node);
			} else {
				// create a bounding volume of the mesh and split it
				BV bv = bvFactory.makeBV( mesh );
				tree->setBV( bv , node);
				//std::cout << "Range: "<< mesh.getGlobalIndex(0) << ";" << mesh.getGlobalIndex(mesh.getSize()) << std::endl;
				// were to split the mesh (the order in the mesh might be changed in this function)
				size_t k = splitter.partitionMesh(mesh, bv );

				// left child
				if(k>0){
                    typename BVTree::node_iterator leftnode = tree->createLeft( node );
                    recursiveTopDownTree(tree, leftnode, mesh.getSubRange(0,k), bvFactory, splitter, maxTrisInLeaf);
				}

				// right child
				if(k<mesh.getSize()){
                    typename BVTree::node_iterator rightnode = tree->createRight( node );
                    recursiveTopDownTree(tree, rightnode, mesh.getSubRange(k,mesh.getSize()), bvFactory, splitter, maxTrisInLeaf);
				}
			}
		}

	};
}
}

#endif /*OBBTREE_HPP_*/
