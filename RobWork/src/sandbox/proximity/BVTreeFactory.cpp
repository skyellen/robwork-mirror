#include "BVTreeFactory.hpp"
#include <sandbox/Covariance.hpp>

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

	struct TriCenterIterator {
		Vector3D<> _pos;
		const TriMesh& _mesh;
		size_t _first,_end;
		bool _useAreaWeight;
		TriCenterIterator(const TriMesh& mesh, bool useAreaWeight=false):
			_mesh(mesh),_first(0),_end(mesh.getSize()),_useAreaWeight(useAreaWeight)
		{}

		Vector3D<>& operator*() {
        	return _pos;
        }

        Vector3D<>* operator->() { return &_pos; }

        TriCenterIterator& operator++(){ inc(); return *this; }

        bool operator==(const TriCenterIterator& other) const{ return _first == other._end;}
        bool operator!=(const TriCenterIterator& other) const { return _first < other._end;}

        void inc(){
        	++_first;
        	if(_first!=_end){
				Triangle<> tri = _mesh.getTriangle(_first);
				if(_useAreaWeight){
					double area = tri.calcArea();
					_pos = area*(tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0;
				} else {
					_pos = (tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0;
				}
        	}
        }


	};

	struct OBBFactory: public BVTreeFactory::BVFactory<OBB<> > {


		//! @brief create a BV
		virtual OBB<> makeBV(const rw::geometry::TriMesh& mesh){
			Covariance<> covar;
			TriCenterIterator iter(mesh, false);
			covar.doInitialize<TriCenterIterator,3>(iter,iter);
			EigenDecomposition<> eigend = covar.eigenDecompose();
			// the eigendecomposition has the eigen vectors and value.
			// we want the x-axis of the OBB to be aligned with the largest eigen vector.
			//std::cout  << "EigenValues:  " <<  eigend.getEigenValues() << std::endl;
			eigend.sort();
			//std::cout  << "EigenValues:  " <<  eigend.getEigenValues() << std::endl;
			Vector3D<> axisX( eigend.getEigenVector(2) );
			Vector3D<> axisY( eigend.getEigenVector(1) );
			Vector3D<> axisZ = cross(axisX,axisY);
			// so now we can form the basis of the rotation matrix of the OBB
			Rotation3D<> rot(normalize(axisX),normalize(axisY),normalize(axisZ));
			Rotation3D<> rotInv = inverse( rot );
			// last we need to find the maximum and minimum points in the mesh to determine
			// the bounds (halflengts) of the OBB
	        Triangle<> t = mesh.getTriangle(0);
	        Vector3D<> p = rotInv * t[0];
	        Vector3D<> max=p, min=p;
			for(size_t i=1;i<mesh.getSize();i++){
	            Triangle<> tri = mesh.getTriangle(i);
	            for(int pidx=0;pidx<3; pidx++){
                    Vector3D<> p = rotInv * tri[pidx];
                    for(int j=0; j<3; j++){
                        if( p(j)>max(j) ) max(j) = p(j);
                        else if( p(j)<min(j) ) min(j) = p(j);
                    }
	            }
			}
	        Vector3D<> midPoint = rot*( 0.5*(max+min));
	        Vector3D<> halfLength = 0.5*(max-min);
	        //std::cout << "halflength: " << halfLength << std::endl;
	        //std::cout << "midpoint: " << midPoint << std::endl;
	        Transform3D<> trans(midPoint,rot);
	        //std::cout << "Trans mid: " << trans.P() << std::endl;
	        return OBB<>(trans, halfLength);

		}


	};


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
	return rw::common::ownedPtr( new OBBFactory() );
}
