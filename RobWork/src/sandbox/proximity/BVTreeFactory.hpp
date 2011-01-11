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
			return makeTopDownTree(mesh, *bvfactory, *splitter, maxTrisInLeaf);
		}

		BinaryBVTree<rw::geometry::OBB<> >* makeTopDownOBBTreeCovarMean(rw::geometry::TriMesh::Ptr mesh, int maxTrisInLeaf=1){
			rw::common::Ptr<BVFactory<rw::geometry::OBB<> > > bvfactory = makeOBBCovarFactory();
			rw::common::Ptr<BVSplitterStrategy<rw::geometry::OBB<> > > splitter = makeOBBMeanSplitter();
			return makeTopDownTree(mesh, *bvfactory, *splitter, maxTrisInLeaf);
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
		template<class BV>
		static BinaryBVTree<BV>* makeTopDownTree(rw::geometry::TriMesh::Ptr mesh,
							BVFactory<BV>& bvFactory,
							BVSplitterStrategy<BV>& splitter,
							int maxTrisInLeaf=1){
			using namespace rw::math;
			using namespace rw::geometry;

			// we create the binary tree
			BinaryBVTree<BV>* tree = new BinaryBVTree<BV>();

			// create a proxy for the triangle mesh
			IndexedTriArray<> idxArray(mesh);

			// now for each tri soup indicated by the triangle indexes compute a OBB sub tree
			typename BinaryBVTree<BV>::Node **root = tree->getRoot();
			recursiveTopDownTree<BV>(tree, root, idxArray, bvFactory, splitter, maxTrisInLeaf);

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
		template<class BV>
		static void recursiveTopDownTree(BinaryBVTree<BV>* tree,
									typename BinaryBVTree<BV>::Node **node,
									rw::geometry::IndexedTriArray<> mesh,
									BVFactory<BV>& bvFactory,
									BVSplitterStrategy<BV> &splitter,
									size_t maxTrisInLeaf){

			if(mesh.getSize()==0){
				*node = NULL;
			} else if(mesh.getSize()<=maxTrisInLeaf){
				// make a leaf node
				*node = tree->createNode();
				(*node)->primIdx() = mesh.getGlobalIndex(0);
				(*node)->setNrOfPrims(mesh.getSize());
			} else {
				// create a bounding volume of the mesh and split it
				*node = tree->createNode();
				//std::cout << (int)(*node)->nrOfPrims() << std::endl;
				(*node)->bv() = bvFactory.makeBV( mesh );

				// were to split the mesh (the order in the mesh might be changed in this function)
				size_t k = splitter.partitionMesh(mesh, (*node)->bv() );

				// left child
				recursiveTopDownTree(tree, (*node)->left(), mesh.getSubRange(0,k), bvFactory, splitter, maxTrisInLeaf);
				// right child
				recursiveTopDownTree(tree, (*node)->right(), mesh.getSubRange(k,mesh.getSize()), bvFactory, splitter, maxTrisInLeaf);
			}
		}

	};
}
}

#endif /*OBBTREE_HPP_*/
