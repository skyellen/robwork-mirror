#include "BVTreeFactory.hpp"
#include <rw/geometry/Covariance.hpp>
#include <rw/math/EigenDecomposition.hpp>
#include <rw/common/Timer.hpp>
#include <float.h>
using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::math;
using namespace rw::common;

namespace {

    /**
     * @brief computes a score of a specific splitting axis. Basically each triangle in the mesh is
     * visited and all vertices are tested for being on either left or right side of the splitting
     * value. The score is then the total number of triangles being completely on either left or right
     * side. Which means that a score=mesh.size() is the best score where the split value completely splits
     * the triangles into two volumes. score=0 is the worst where all triangles has vertices on both sides on the split value.
     * @param mesh
     * @param splitValue
     * @param t3d
     * @return
     */
    int evaluateSplitAxis(rw::geometry::IndexedTriArray<>& mesh, int splitAxis, double splitValue, const Transform3D<>& t3d){
        int left=0, right=0;
        for(size_t i=0; i<mesh.getSize(); i++ ){

            // transform the vertex point to the obb root
            Triangle<> tri = mesh.getTriangle(i);
            bool toLeft = (t3d*tri.getVertex(0))[splitAxis] < splitValue;
            if(toLeft){
                // check if its really to the left
                toLeft &= (t3d*tri.getVertex(1))[splitAxis] < splitValue;
                toLeft &= (t3d*tri.getVertex(2))[splitAxis] < splitValue;
                if(toLeft==true) left++;
            } else {
                toLeft |= (t3d*tri.getVertex(1))[splitAxis] < splitValue;
                toLeft |= (t3d*tri.getVertex(2))[splitAxis] < splitValue;
                if(toLeft==false) right++;
            }
        }
        return right+left;
    }

	/**
	 * @brief Object median splitting strategy using axis of largest variance. Splits the mesh
	 * in the median on the axis with largest variance.
	 */
	struct OBBMedianSplitter: public BVTreeFactory::BVSplitterStrategy<OBB<> > {
	public:
		size_t partitionMesh(rw::geometry::IndexedTriArray<>& mesh, OBB<> obb){

			Transform3D<> t3d = inverse( obb.getTransform() );

			int splitAxis = 0, bestSplitAxis = 0; // choose longest (x-axis) for splitting the Box
			int bestSplitScore =0;
			int median = (int)(mesh.getSize()/2);
			Timer time;
			do{
				// calculate median
				// sort all indexes in trisIdx
				mesh.sortAxis(splitAxis, t3d);

				// now make sure that the choosen split axis is not a bad one
				Triangle<> tri = mesh.getTriangle(median);
				Vector3D<> center = t3d*((tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0);
				int score = evaluateSplitAxis(mesh, splitAxis, center[splitAxis], t3d);

				if(score>bestSplitScore){
					bestSplitScore = score;
					bestSplitAxis = splitAxis;
				}

				// criteria for an okay splitting point
				if( score + mesh.getSize()/8 >= mesh.getSize() ){
					break;
				}

				splitAxis++;
				if(splitAxis==3){
					// No axis was goood, so we use the best one
					if(bestSplitAxis!=2)
						mesh.sortAxis(bestSplitAxis, t3d);
					break;
				}

			} while( splitAxis<3 );

			return median;
		}
	};

    /**
     * @brief Spatial Median splitting strategy. The median of the bounding volume projection
     * extends are used as splitting point.
     */
    struct OBBMeanSplitter: public BVTreeFactory::BVSplitterStrategy<OBB<> > {
    public:
        size_t partitionMesh(rw::geometry::IndexedTriArray<>& mesh, OBB<> obb){
            Transform3D<> t3d = inverse( obb.getTransform() );

            int splitAxis = 0, bestSplitAxis = 0; // choose longest (x-axis) for splitting the Box
            int bestSplitScore =0;
            double mean = 0;
            do{
                // We need to calculate the mean for a splitting axis
                mean=0;
                for(size_t i=0; i<mesh.getSize(); i++ ){
                    // transform the vertex point to the obb root
                    Triangle<> tri = mesh.getTriangle(i);
                    Vector3D<> center = t3d*((tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0);
                    mean += center[splitAxis];
                }
                mean = mean/mesh.getSize();

                int score = evaluateSplitAxis(mesh, splitAxis, mean, t3d);
                if(score>bestSplitScore){
                    bestSplitScore = score;
                    bestSplitAxis = splitAxis;
                }

                // criteria for an okay splitting point
                if( score + mesh.getSize()/8 >= mesh.getSize() ){
                    break;
                } else {
                    splitAxis++;
                }

            } while( splitAxis<3 );

            unsigned int meanSplit = 0;
            double mindist = DBL_MAX;
            // find the mean split
            // now we sort the best split axis
            mesh.sortAxis(bestSplitAxis, t3d);
            // and find the splitting index
            for(size_t i=0; i<mesh.getSize(); i++ ){
                // transform the vertex point to the obb root
                Triangle<> tri = mesh.getTriangle(i);
                Vector3D<> center = t3d*((tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0);
                if( fabs(center[bestSplitAxis]-mean)<mindist ){
                    mindist = fabs(center[bestSplitAxis]);
                    meanSplit = i;
                }
            }

            if(meanSplit==0)
                return 1;
            if(meanSplit==mesh.size()-1)
                return meanSplit-1;
            return meanSplit;
        }
    };

	/**
	 * @brief Spatial Median splitting strategy. The median of the bounding volume projection
	 * extends are used as splitting point.
	 */
	struct OBBSpatialMedianSplitter: public BVTreeFactory::BVSplitterStrategy<OBB<> > {
	public:
		size_t partitionMesh(rw::geometry::IndexedTriArray<>& mesh, OBB<> obb){
		    //std::cout << "partition mesh: " << mesh.size() << std::endl;
			Transform3D<> t3d = inverse( obb.getTransform() );

			size_t splitAxis = 0, bestSplitAxis = 0; // choose longest (x-axis) for splitting the Box
			size_t bestSplitScore =0;

			do{
			    unsigned int score = evaluateSplitAxis(mesh, splitAxis, 0, t3d);
				if(score>bestSplitScore){
					bestSplitScore = score;
					bestSplitAxis = splitAxis;
				}

				// criteria for an okay splitting point
				if( score + mesh.getSize()/8 >= mesh.getSize() ){
					break;
				} else {
					splitAxis++;
				}

			} while( splitAxis<3 );
			//std::cout << "Split axis: " << bestSplitAxis << std::endl;
			double mindist = DBL_MAX;
			unsigned int closest = 0;
			// now we sort the best split axis
			mesh.sortAxis(bestSplitAxis, t3d);
			// and find the splitting index
            for(size_t i=0; i<mesh.getSize(); i++ ){
                // transform the vertex point to the obb root
                Triangle<> tri = mesh.getTriangle(i);
                Vector3D<> center = t3d*((tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0);
                if( fabs(center[bestSplitAxis])<mindist ){
                    mindist = fabs(center[bestSplitAxis]);
                    closest = i;
                }
            }
            //std::cout << "closest: " << closest << std::endl;
            if(closest==0)
                return 1;
            if(closest==mesh.size()-1)
                return closest-1;
			return closest;
		}
	};


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
		    return OBB<>::buildTightOBB(mesh);

		    //std::cout << "\nMesh size: " << mesh.size() << "\n";
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
                //if(mesh.getSize()<5)
                //    std::cout << "TRI\n";

	            for(int pidx=0;pidx<3; pidx++){
                    Vector3D<> p = rotInv * tri[pidx];
                  //  if(mesh.getSize()>3)
                  //      std::cout << "-- " << p << "\n";

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
	return rw::common::ownedPtr( new OBBSpatialMedianSplitter() );
}

rw::common::Ptr<BVTreeFactory::BVSplitterStrategy<rw::geometry::OBB<> > > BVTreeFactory::makeOBBMeanSplitter(){
	return rw::common::ownedPtr( new OBBMeanSplitter() );
}

rw::common::Ptr<BVTreeFactory::BVFactory<rw::geometry::OBB<> > > BVTreeFactory::makeOBBCovarFactory(){
	return rw::common::ownedPtr( new OBBFactory() );
}
