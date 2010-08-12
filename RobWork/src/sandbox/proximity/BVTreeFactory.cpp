#include "BVTreeFactory.hpp"

using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::math;

namespace {

	/**
	 * @brief Object median splitting strategy
	 */
	struct OBBMedianSplitter: public BVTreeFactory::BVSplitterStrategy<OBB<> > {
	public:
		size_t partitionMesh(rw::geometry::IndexedTriArray<>& mesh, OBB<> obb){

			Transform3D<> t3d = inverse( obb.getTransform() );

			size_t splitAxis = 0, bestSplitAxis = 0; // choose longest (x-axis) for splitting the Box
			size_t bestSplitScore =0;
			int median = (int)(mesh.getSize()/2);

			do{
				// calculate median
				// sort all indexes in trisIdx
				//std::cout << "Sort " << std::endl;
				mesh.sortAxis(splitAxis, t3d);

				// now make sure that the choosen split axis is not a bad one

				Triangle<> tri = mesh.getTriangle(median);
				Vector3D<> center = (tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0;
				int left=0, right=0;
				for(size_t i=0; i<mesh.getSize(); i++ ){
					// transform the vertex point to the obb root
					tri = mesh.getTriangle(i);
					bool toLeft = tri.getVertex(0)[splitAxis] < center[splitAxis];
					if(toLeft){
						// check if its really to the left
						toLeft &= tri.getVertex(1)[splitAxis] < center[splitAxis];
						toLeft &= tri.getVertex(2)[splitAxis] < center[splitAxis];
						if(toLeft==true) left++;
					} else {
						toLeft |= tri.getVertex(1)[splitAxis] < center[splitAxis];
						toLeft |= tri.getVertex(2)[splitAxis] < center[splitAxis];
						if(toLeft==false) right++;
					}
				}

				size_t score = right + left;
				if(score>bestSplitScore){
					bestSplitScore = score;
					bestSplitAxis = splitAxis;
				}

				// criteria for an okay splitting point
				if( right + left + mesh.getSize()/8 >= mesh.getSize() ){
					break;
				} else {
					splitAxis++;
				}

				if(splitAxis==3){
					// use best split axis
					if(bestSplitAxis!=2)
						mesh.sortAxis(bestSplitAxis, t3d);

					break;
				}

			} while( splitAxis<3 );

			return median;
		}
	};


	/**
	 * @brief Object median splitting strategy
	 */
/*
	struct OBBSpatialMedianSplitter: public BVTreeFactory::BVSplitterStrategy<OBB<> > {
	public:
		size_t partitionMesh(rw::geometry::IndexedTriArray<>& mesh, OBB<> obb){

			Transform3D<> t3d = inverse( obb.getTransform() );

			size_t splitAxis = 0, bestSplitAxis = 0; // choose longest (x-axis) for splitting the Box
			size_t bestSplitScore =0;

			do{
				double median = obb.getHalfLengths()[splitAxis];

				Triangle<> tri = mesh.getTriangle(median);
				Vector3D<> center = (tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0;
				int left=0, right=0;
				for(size_t i=0; i<mesh.getSize(); i++ ){
					// transform the vertex point to the obb root
					tri = mesh.getTriangle(i);
					bool toLeft = tri.getVertex(0)[splitAxis] < center[splitAxis];
					if(toLeft){
						// check if its really to the left
						toLeft &= tri.getVertex(1)[splitAxis] < center[splitAxis];
						toLeft &= tri.getVertex(2)[splitAxis] < center[splitAxis];
						if(toLeft==true) left++;
					} else {
						toLeft |= tri.getVertex(1)[splitAxis] < center[splitAxis];
						toLeft |= tri.getVertex(2)[splitAxis] < center[splitAxis];
						if(toLeft==false) right++;
					}
				}

				// lower score is best,
				size_t score = std::abs(right - left);
				if(score<bestSplitScore){
					bestSplitScore = score;
					bestSplitAxis = splitAxis;
				}

				// criteria for an okay splitting point
				if( right + left + mesh.getSize()/8 >= mesh.getSize() ){
					break;
				} else {
					splitAxis++;
				}

				if(splitAxis==3){
					// use best split axis
					if(bestSplitAxis!=2)
						mesh.sortAxis(bestSplitAxis, t3d);

					break;
				}

			} while( splitAxis<3 );

			mesh.sortAxis(splitAxis, t3d);

			return median;
		}
	};
*/

}

rw::common::Ptr<BVTreeFactory::BVSplitterStrategy<rw::geometry::OBB<> > > BVTreeFactory::makeOBBMedianSplitter(){
	return rw::common::ownedPtr( new OBBMedianSplitter() );
}

rw::common::Ptr<BVTreeFactory::BVSplitterStrategy<rw::geometry::OBB<> > > BVTreeFactory::makeOBBSpatialMedianSplitter(){
	return rw::common::ownedPtr( new OBBMedianSplitter() );
}

rw::common::Ptr<BVTreeFactory::BVSplitterStrategy<rw::geometry::OBB<> > > BVTreeFactory::makeOBBMeanSplitter(){
	return rw::common::ownedPtr( new OBBMedianSplitter() );
}

rw::common::Ptr<BVTreeFactory::BVFactory<rw::geometry::OBB<> > > BVTreeFactory::makeOBBCovarFactory(){

}
