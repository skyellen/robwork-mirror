#ifndef OBBTREE_HPP_
#define OBBTREE_HPP_


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

namespace rw {
namespace geometry {


	class OBBTreeFactory {
	public:

		/**
		 * @brief general function for constructing a binary bounding volume tree in a top down fashion.
		 *
		 *
		 *
		 * @param mesh [in] the mesh on which to construct the bounding volume tree
		 * @param bvFactory [in] a factory for creating/fitting bounding volumes given a triangle mesh
		 * @param splitter [in] divides a mesh into 2 meshes by sorting the mesh and providing a splitting index
		 * @param maxTrisInLeaf [in] the maximum number of tris that are allowed in each leaf node
		 * @return
		 */
		template<class BV, class BV_CALCULATOR, class SPLITTER_STRATEGY>
		static BinaryTreeG<BV>* makeTopDownTree(const TriMesh& mesh, BV_CALCULATOR& bvFactory, SPLITTER_STRATEGY &splitter, int maxTrisInLeaf=1){
			using namespace rw::math;
			using namespace rw::geometry;

			// first build an BV around the whole geometry and initialize the binary tree

			BinaryTreeG<BV>* tree = new BinaryTreeG<BV>();

			// next order the the triangles according to the largest axis in the box (x-axis)
			std::vector<int> trisIdx( mesh.getSize() );
			for(size_t i=0; i< trisIdx.size(); i++)
				trisIdx[i] = i;

			IndexedTriArray<> idxArray(mesh, trisIdx);

			// now for each tri soup indicated by the triangle indexes compute a OBB sub tree
			recursiveTopDownTree<BV,BV_CALCULATOR,SPLITTER_STRATEGY>(tree, &tree->getRoot(), mesh, trisIdx, bvFactory, splitter, maxTrisInLeaf);
		}

	private:
		template<class BV, class BV_CALCULATOR, class SPLITTER_STRATEGY>
		static void recursiveTopDownTree(BinaryTreeG<BV>* tree, typename BinaryTreeG<BV>::Node **node, IndexedTriArray<> &mesh, BV_CALCULATOR bvFactory, SPLITTER_STRATEGY &splitter, size_t maxTrisInLeaf){
			if(mesh.getSize()==0){
				*node = NULL;
			} else if(mesh.getSize()<=maxTrisInLeaf){
				// make a leaf node
				*node = tree->createNode();
				(*node)->primIdx() = mesh.getGlobalIndex(0);
				(*node)->nrOfPrims = mesh.getSize();
			} else {
				// create a bounding volume of the mesh and split it
				*node = tree->createNode();
				(*node)->bv() = bvFactory.makeBV( mesh );

				// were to split the mesh (the order in the mesh might be changed in this function)
				size_t k = splitter.partitionMesh(mesh, (*node)->bv() );

				// left child
				recursiveTopDownTree(tree, &(*node)->left(), mesh.getSubRange(0,k), bvFactory, splitter );
				// right child
				recursiveTopDownTree(tree, &(*node)->right(), mesh.getSubRange(k,mesh.getSize()), bvFactory, splitter );
			}
		}

	};


#ifdef NOT_TEMPLATED_STUFF

	  static OBBTree* buildRecMedian(int n, const OBB<T>& obb,
				const IdxTriMesh& mesh,
				std::vector<int>& trisIdx,
				TimingState &tstate,
				int tDegree,
				size_t maxTrisInLeaf)


	template <class T=double>
	class OBBTree {
		private:

			typedef std::vector<rw::math::Vector3D<T> > VertexArray;
			OBB<T> _obb;
			int _treeDegree;
			std::vector<OBBTree<T>* > _children;
			std::vector<int> _tris;
			int _index;

			OBBTree(OBB<T> obb, std::vector<OBBTree<T>*> children, int tDegree, int index):
				_obb(obb),_children(children),_treeDegree(tDegree),_index(index)
		{
		}

		public:

			int getIndex(){return _index;};

			virtual ~OBBTree(){
				BOOST_FOREACH(OBBTree<T>* child, _children){
					//std::cout << "D " << child << std::endl;
					delete child;
				}
			}

			int getNrOfChildren() const { return _children.size(); };

			int getMaxNrOfChildren(){ return std::floor( std::pow(2.0,_treeDegree) );};

			const std::vector<OBBTree<T>* >& getChildren() const {
				return _children;
			}

			std::vector<OBBTree<T>* >& getChildren() {
				return _children;
			}

			const OBB<T>& getOBB() const {
				return _obb;
			}

			OBB<T>& getOBB() {
				return _obb;
			}

			void setTris(const std::vector<int>& tris){
				_tris = tris;
			}

			const std::vector<int>& getTris(){
				return _tris;
			}


			int getNrOfBVs()  const {
				return _children.size();
			}

			int getNrOfPrimitives()  const {
				return _tris.size();
			}

			bool isLeaf() const {
				return _children.size()==0;
			}

		public:
			typedef rw::geometry::IndexedTriMesh<T> IdxTriMesh;

			struct TimingState {
				rw::common::Timer totalTime;
				rw::common::Timer sortTime;
				rw::common::Timer fitTime;
				int nrOfFits;
			};

			static OBBTree* Build(size_t n, const IdxTriMesh& mesh, int maxTrisInLeaf=1){
				using namespace rw::math;
				using namespace rw::geometry;

				if( n<=0 )
					RW_THROW("Cannot create OBB tree with n==0, ie. only one child per parent");

				int tDegree = n;

				// first build an OBB around the whole geometry
				OBB<T> obb = OBB<T>::buildTightOBB1( mesh );

				// next order the the triangles according to the largest axis in the box (x-axis)
				std::vector<int> trisIdx(mesh.size());
				for(size_t i=0; i< trisIdx.size(); i++)
					trisIdx[i] = i;

				//printf("mesh.size() = %d\n", mesh.size());
				// assert(0);

				// now for each tri soup indicated by the triangle indexes compute a OBB sub tree
				TimingState tState;
				tState.fitTime.pause();
				tState.sortTime.pause();
				tState.nrOfFits = 1;

				OBBTree *root = buildRecMedian(0, obb, mesh, trisIdx, tState, tDegree, maxTrisInLeaf);



	/*
				std::cout << "Build timings: " << std::endl
						  << "- Total : " << tState.totalTime.getTime() << std::endl
						  << "- Fit   : " << tState.fitTime.getTime() << std::endl
						  << "- Sort  : " << tState.sortTime.getTime() << std::endl
						  << "- # Fits: " << tState.nrOfFits << std::endl;
	*/
	//			std::cout << "Build rec finished" << std::endl;
				int N = std::floor( std::pow(2.0, (int)n) );
				const int lastLevel = (int)std::ceil( std::log(N*(mesh.size()/maxTrisInLeaf)-1)/std::log(N) );
				int maxNrBvs = std::max((int)std::floor( std::pow((double)N, lastLevel))-2,0)+1;
	//			std::cout << "maxTrisInLeaf: " << maxTrisInLeaf << std::endl;
	//			std::cout << "lastLevel: " << lastLevel << std::endl;
	//			std::cout << "maxNrBvs: " << maxNrBvs << std::endl;
				RW_ASSERT( tState.nrOfFits<=maxNrBvs );

				return root;

#ifdef MULTIPLE_DIM_TREES
// this was for trees of multiple dimensions


	//			std::cout << "Changing tree degree!" << std::endl;
				// else restructure the tree where each parent has 2^n children
				typedef std::pair<int,OBBTree*> ParentType;
				std::stack<ParentType> stack;
				std::vector<OBBTree*> children;
				std::vector<OBBTree*> parents;
				std::vector<OBBTree*> leafs;
				stack.push( std::make_pair(0,root) );
				//parent.push_back(root);
				while(!stack.empty()){
					size_t lvl = stack.top().first;
					OBBTree* parent = stack.top().second;
					stack.pop();
					if( parent==NULL )
						continue;

					children = parent->getChildren();
					lvl++;
					// now locate all children that should be in this parent
					while( (lvl) % n ) {
						parents = children;
						children.clear();
						BOOST_FOREACH(OBBTree* p, parents){
							if( p==NULL )
								continue;

							if( p->getChildren().size() == 0 ){
								children.push_back(p);
								continue;
							}
							BOOST_FOREACH(OBBTree* child, p->getChildren() ){
								children.push_back(child);
							}
						}
						lvl++;
					}
					// copy children vector into the current parent
					parent->getChildren() = children;
					// add all children to stack
					BOOST_FOREACH(OBBTree* child, children ){
						stack.push( std::make_pair(lvl, child) );
					}
				}
				return root;
#endif

			};

		private:

			struct TrisIdxSort
			{
				TrisIdxSort(
					 const int splitAxis,
					 const rw::math::Transform3D<T>& t3d,
					 const IdxTriMesh& mesh):
					_splitAxis(splitAxis),
					_t3d(t3d),
					_mesh(mesh)
				{}

				bool operator()(const int& i0, const int& i1) {
					using namespace rw::math;
					using namespace rw::geometry;
					const std::vector<Vector3D<T> > &verts = _mesh.getVertices();
					const IndexedTriangle<T> &tri = _mesh[ i0 ];
					Vector3D<T> c0 = _t3d*((verts[ tri[0] ]+verts[ tri[1] ]+verts[ tri[2] ])/3.0);
					const IndexedTriangle<T> &t1 = _mesh[ i1 ];
					Vector3D<T> c1 = _t3d*((verts[ t1[0] ]+verts[ t1[1] ]+verts[ t1[2] ])/3.0);

					return  c0(_splitAxis)<c1(_splitAxis);
				}
				const int _splitAxis;
				const rw::math::Transform3D<T> _t3d;
				const IdxTriMesh& _mesh;

			};


			/**
			 * @brief creates a OBBTree of the triangles refereced by the indexes in tris
			 */
			/*	static OBBTree* BuildRec(int n, const OBB<T>& obb,
				const rw::geometry::IndexedTriMesh<T,rw::geometry::N0>& mesh,
				const std::vector<rw::geometry::IndexedTriangle<rw::geometry::N0> >& tris)
				*/
			static OBBTree* BuildRec(int n, const OBB<T>& obb,
					const IdxTriMesh& mesh,
					std::vector<int>& trisIdx,
					int maxTrisInLeaf=1)
			{
				using namespace rw::math;
				using namespace rw::geometry;
				//std::cout << "Build rec: " << trisIdx.size() << std::endl;
				//std::vector<int> trisIdxSorted(triIdx.size());

				typedef IndexedArray<Vector3D<T> > VertexArray;

				std::vector<OBBTree<T>*> children;


				// if number of tris is less than .. create leaf
				if( trisIdx.size()<=maxTrisInLeaf){
					OBBTree<T> *tree = new OBBTree<T>(obb, children);
					tree->setTris( trisIdx );
					return tree;
				}

				// extract vertices from the tri indexes, and make sure to
				const std::vector<Vector3D<T> > &verts = mesh.getVertices();
				std::vector<int> lowVertIdx,highVertIdx;
				//std::vector<IndexedTriangle<N0> > lowTris,highTris;
				std::vector<int> lowTrisIdx,highTrisIdx;
				Transform3D<T> t3d = inverse( obb.getTransform() );
				size_t splitAxis = 0; // choose longest (x-axis) for splitting the Box

				do{
					// calculate median
					// sort all indexes in trisIdx
					//std::cout << "Sort " << std::endl;
					std::sort(trisIdx.begin(), trisIdx.end(), TrisIdxSort(splitAxis, t3d, mesh));
					// now make sure that the choosen split axis is not a bad one
					int median = (int)(trisIdx.size()/2);
					//std::cout << "Median: " << median << std::endl;
					for(size_t i=0; i<trisIdx.size(); i++ ){
						// transform the vertex point to the obb root
						const IndexedTriangle<T> &tri = mesh[ trisIdx[i] ];
						//std::cout << "C: " << c << std::endl;
						// we use median
						if( i<median ){// add tri to lower tris
							lowTrisIdx.push_back( trisIdx[i] );
							lowVertIdx.push_back( tri[0] );
							lowVertIdx.push_back( tri[1] );
							lowVertIdx.push_back( tri[2] );
						} else {// add tri to higher tris
							highTrisIdx.push_back( trisIdx[i] );
							highVertIdx.push_back( tri[0] );
							highVertIdx.push_back( tri[1] );
							highVertIdx.push_back( tri[2] );
						}

					}
	/*
					for(size_t i=0; i<trisIdx.size(); i++ ){
						// transform the vertex point to the obb root
						const IndexedTriangle &tri = mesh[ trisIdx[i] ];
						Vector3D<T> p = verts[ tri[0] ];
						Vector3D<T> q = verts[ tri[1] ];
						Vector3D<T> r = verts[ tri[2] ];
						Vector3D<T> cTmp = ((p+q+r)/3.0);
						Vector3D<T> c = t3d * cTmp;

						//std::cout << "C: " << c << std::endl;
						if( c[splitAxis]<0 ){// add tri to lower tris
							lowTrisIdx.push_back( trisIdx[i] );
							lowVertIdx.push_back( tri[0] );
							lowVertIdx.push_back( tri[1] );
							lowVertIdx.push_back( tri[2] );
						} else {// add tri to higher tris
							highTrisIdx.push_back( trisIdx[i] );
							highVertIdx.push_back( tri[0] );
							highVertIdx.push_back( tri[1] );
							highVertIdx.push_back( tri[2] );
						}
					}
					*/
					// handle if all vertices lie on one side of the splitting plane
					//std::cout << "Tris: " << highTrisIdx.size() << " " << lowTrisIdx.size() << std::endl;

					if(splitAxis==2)
						break;

					if( highTrisIdx.size() == trisIdx.size() ||
							lowTrisIdx.size() == trisIdx.size() ){
						highTrisIdx.clear();
						highVertIdx.clear();
						lowTrisIdx.clear();
						lowVertIdx.clear();
						splitAxis++;
						//std::cout << "Try next split axis.. " << std::endl;
					} else {
						break;
					}
				} while( splitAxis<3 );

				RW_ASSERT(lowTrisIdx.size()+highTrisIdx.size()==trisIdx.size());

				//std::cout << "Before test Build both: " << std::endl;
				if( lowTrisIdx.size() == 0 ) {
					// this is a leaf node, add highVertIdx tris to obb
					VertexArray highVerts(&verts, highVertIdx);
					OBB<T> lowobb = OBB<T>::template BuildTightOBB<VertexArray>(highVerts, n);
					OBBTree<T> *tree = new OBBTree<T>(lowobb,children);
					tree->setTris( highTrisIdx );
					return tree;
				}

				if( highTrisIdx.size() == 0 ){
					// this is a leaf node, add highVhyertIdx tris to obb
					VertexArray lowVerts(&verts, lowVertIdx);
					OBB<T> highobb = OBB<T>::template BuildTightOBB<VertexArray>(lowVerts, n);
					OBBTree<T> *tree = new OBBTree<T>(highobb,children);
					tree->setTris( lowTrisIdx );
					return tree;
				}

				//std::cout << "Build left OBB" << std::endl;
				VertexArray lowVerts(&verts, lowVertIdx);
				OBB<T> leftObb = OBB<T>::template BuildTightOBB<VertexArray>(lowVerts,n);
				OBBTree *left = BuildRec((n+1)*2, leftObb, mesh, lowTrisIdx);

				//std::cout << "Build right OBB" << std::endl;
				VertexArray highVerts(&verts, highVertIdx);
				OBB<T> rightObb = OBB<T>::template BuildTightOBB<VertexArray>(highVerts,n+1);
				OBBTree *right = BuildRec((n+1)*2+2,rightObb,mesh,highTrisIdx);

				//if(right==NULL && left==NULL)
				children.push_back(left);
				children.push_back(right);
				return new OBBTree<T>(obb,children);
			}



			  static OBBTree* buildRecMedian(int n, const OBB<T>& obb,
						const IdxTriMesh& mesh,
						std::vector<int>& trisIdx,
						TimingState &tstate,
						int tDegree,
						size_t maxTrisInLeaf)
				{
					using namespace rw::math;
					using namespace rw::geometry;
					//std::cout << "Build rec: " << trisIdx.size() << std::endl;
					//std::vector<int> trisIdxSorted(triIdx.size());

					//typedef IndexedArray<Vector3D<T> > VertexArray;

					std::vector<OBBTree<T>*> children;


					// if number of tris is less than .. create leaf
					if( trisIdx.size()<=maxTrisInLeaf){
						OBBTree<T> *tree = new OBBTree<T>(obb, children, tDegree, n);
						tree->setTris( trisIdx );
						return tree;
					}

					// extract vertices from the tri indexes, and make sure to
					//const std::vector<Vector3D<T> > &verts = mesh.getVertices();

					//std::vector<IndexedTriangle<N0> > lowTris,highTris;
					std::vector<int> lowTrisIdx,highTrisIdx;
					Transform3D<T> t3d = inverse( obb.getTransform() );
					size_t splitAxis = 0; // choose longest (x-axis) for splitting the Box

					tstate.sortTime.resume();

					do{
						// calculate median
						// sort all indexes in trisIdx
						//std::cout << "Sort " << std::endl;
						std::sort(trisIdx.begin(), trisIdx.end(), TrisIdxSort(splitAxis, t3d, mesh));
						// now make sure that the choosen split axis is not a bad one

						size_t nrTris = trisIdx.size();
						if(nrTris & 0x1)
							nrTris++;
						size_t median = nrTris>>1;

						//std::cout << "Median: " << median << std::endl;
						for(size_t i=0; i<trisIdx.size(); i++ ){
							// transform the vertex point to the obb root
							//const IndexedTriangle<T> &tri = mesh[ trisIdx[i] ];
							//std::cout << "C: " << c << std::endl;
							// we use median
							if( i<median ){// add tri to lower tris
								lowTrisIdx.push_back( trisIdx[i] );
							} else {// add tri to higher tris
								highTrisIdx.push_back( trisIdx[i] );
							}

						}
						// handle if all vertices lie on one side of the splitting plane
						//std::cout << "Tris: " << highTrisIdx.size() << " " << lowTrisIdx.size() << std::endl;

						if(splitAxis==2)
							break;

						if( highTrisIdx.size() == trisIdx.size() ||
								lowTrisIdx.size() == trisIdx.size() ){
							highTrisIdx.clear();
							lowTrisIdx.clear();
							splitAxis++;
							//std::cout << "Try next split axis.. " << std::endl;
						} else {
							break;
						}
					} while( splitAxis<3 );

					tstate.sortTime.pause();

					RW_ASSERT(lowTrisIdx.size()+highTrisIdx.size()==trisIdx.size());

					//std::cout << "Before test Build both: " << std::endl;
					if( lowTrisIdx.size() == 0 ) {
						// this is a leaf node, add highVertIdx tris to obb
						//VertexArray highVerts(&verts, highVertIdx);
						tstate.nrOfFits++;
						IndexedTriArray<> highTris(&mesh, highTrisIdx);
						OBB<T> lowobb = OBB<T>::buildTightOBB1(highTris);
						OBBTree<T> *tree = new OBBTree<T>(lowobb,children,tDegree, n);
						tree->setTris( highTrisIdx );
						return tree;
					}

					if( highTrisIdx.size() == 0 ){
						// this is a leaf node, add highVhyertIdx tris to obb
						// VertexArray lowVerts(&verts, lowVertIdx);
						tstate.nrOfFits++;

						IndexedTriArray<> lowTris(&mesh, lowTrisIdx);
						OBB<T> highobb = OBB<T>::buildTightOBB1(lowTris);
						OBBTree<T> *tree = new OBBTree<T>(highobb,children,tDegree, n);
						tree->setTris( lowTrisIdx );
						return tree;
					}

					//std::cout << "Build left OBB" << std::endl;
					//VertexArray lowVerts(&verts, lowVertIdx);
					IndexedTriArray<> lowTris(&mesh, lowTrisIdx);
					tstate.fitTime.resume();
					OBB<T> leftObb = OBB<T>::buildTightOBB1(lowTris);
					tstate.fitTime.pause();
					tstate.nrOfFits++;
					OBBTree *left = buildRecMedian(n*2+1, leftObb, mesh, lowTrisIdx,tstate,tDegree,maxTrisInLeaf);

					//std::cout << "Build right OBB" << std::endl;
					//VertexArray highVerts(&verts, highVertIdx);
					IndexedTriArray<> highTris(&mesh, highTrisIdx);

					tstate.fitTime.resume();
					OBB<T> rightObb = OBB<T>::buildTightOBB1(highTris,n+1);
					tstate.fitTime.pause();
					tstate.nrOfFits++;

					OBBTree *right = buildRecMedian(n*2+2,rightObb,mesh,highTrisIdx,tstate,tDegree, maxTrisInLeaf);

					//if(right==NULL && left==NULL)
					children.push_back(left);
					children.push_back(right);
					return new OBBTree<T>(obb,children,tDegree, n);
				}

	};



	template <class T=double>
	struct OBBTreeRoot {
	public:
		OBBTreeRoot(OBBTree<T> *tree, rw::math::Transform3D<T> t3d):
			_tree(tree),_transform(t3d),mesh(NULL){}
		OBBTree<T> *_tree;
		rw::math::Transform3D<T> _transform;
		rw::geometry::TriMesh *mesh;

		// stuff
		int triOffset;
		int boxOffset;
	};
#endif
}
}

#endif /*OBBTREE_HPP_*/
