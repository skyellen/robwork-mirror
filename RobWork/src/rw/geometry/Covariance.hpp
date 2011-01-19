/*
 * Covariance3D.hpp
 *
 *  Created on: 04/08/2010
 *      Author: jimali
 */

#ifndef COVARIANCE3D_HPP_
#define COVARIANCE3D_HPP_

#include <vector>

#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/Geometry.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/EigenDecomposition.hpp>
#include <rw/math/Math.hpp>

#include <boost/numeric/ublas/matrix.hpp>



namespace rw {
namespace geometry {

	/**
	 *  @brief class for estimating the covariance of different data
	 */
	template<class T=double>
	class Covariance {
	public:

		Covariance()
		{
		}

		virtual ~Covariance(){};

		const boost::numeric::ublas::matrix<T>& getMatrix(){ return _covar;};

		rw::math::EigenDecomposition<T> eigenDecompose(){
			typedef std::pair<boost::numeric::ublas::matrix<T>,boost::numeric::ublas::vector<T> > ResultType;
			//std::cout << "Covar: " << _covar << std::endl;
			ResultType res = rw::math::LinearAlgebra::eigenDecompositionSymmetric( _covar );
			//std::cout << "res: " <<  res.first << std::endl;
			//std::cout << "res: " <<  res.second << std::endl;
			return rw::math::EigenDecomposition<T>(res.first, res.second);
		}

        /**
         * @brief initialize covariance using a geometry object.
         * @param geom
         */
        void initialize(rw::geometry::Geometry& geom){
            using namespace rw::geometry;
            GeometryData::Ptr data = geom.getGeometryData();
            TriMesh::Ptr mesh = data->getTriMesh(false);

            if( dynamic_cast<IndexedTriMesh<T>*>(mesh.get()) ){
                IndexedTriMesh<T>* imesh = dynamic_cast<IndexedTriMesh<T>*>(mesh.get());
                initialize( imesh->getVertices() );
            } else {
                rw::common::Ptr<IndexedTriMeshN0<T> > ipmesh = TriangleUtil::toIndexedTriMesh< IndexedTriMeshN0<T> >(*mesh);
                initialize( ipmesh->getVertices() );
            }
        }


		void initialize(const std::vector<rw::math::Vector3D<T> >& points){
			//_covar = boost::numeric::ublas::zero_matrix<T>(3, 3);
			doInitialize<typename std::vector<rw::math::Vector3D<T> >::const_iterator, 3>( points.begin(), points.end() );
		}

		template<class RandomAccessIterator, int DIM>
		void doInitialize(RandomAccessIterator first, RandomAccessIterator last){
			using namespace rw::math;
			using namespace boost::numeric;

			//const size_t nrOfPoints = points.size();

			_covar = ublas::zero_matrix<T>(DIM, DIM);

			T centroid[DIM];
			T covarTmp[DIM][DIM];
			for(size_t x=0;x<DIM;x++){
				centroid[x] = 0;
				for(size_t y=0;y<DIM;y++){
					covarTmp[x][y] = 0;
				}
			}
			// we only use triangle centers the vertices directly
			size_t nrOfPoints = 0;
			for( ;first!=last; ++first){

				nrOfPoints++;

				for(size_t j=0; j<DIM; j++)
					centroid[j] += (*first)[j];

				for(size_t j=0; j<DIM; j++)
					for(size_t k=j; k<DIM; k++)
						covarTmp[k][j] += (*first)[k] * (*first)[j];

				/*covarTmp[0][0] += c[0]*c[0];
				covarTmp[1][0] += c[1]*c[0];
				covarTmp[2][0] += c[2]*c[0];
				covarTmp[1][1] += c[1]*c[1];
				covarTmp[2][1] += c[2]*c[1];
				covarTmp[2][2] += c[2]*c[2];
				*/
			}

			for(size_t j=0; j<DIM; j++)
				for(size_t k=j; k<DIM; k++)
					_covar(k,j) = covarTmp[k][j]-centroid[k]*centroid[j]/nrOfPoints;

	/*
			_covar(0,0) = covarTmp[0][0]-centroid[0]*centroid[0]/nrOfPoints;
			_covar(1,0) = covarTmp[1][0]-centroid[1]*centroid[0]/nrOfPoints;
			_covar(2,0) = covarTmp[2][0]-centroid[2]*centroid[0]/nrOfPoints;
			_covar(1,1) = covarTmp[1][1]-centroid[1]*centroid[1]/nrOfPoints;
			_covar(2,1) = covarTmp[2][1]-centroid[2]*centroid[1]/nrOfPoints;
			_covar(2,2) = covarTmp[2][2]-centroid[2]*centroid[2]/nrOfPoints;
	*/

			for(size_t j=1; j<DIM; j++)
				for(size_t k=0; k<j; k++)
					_covar(k,j) = _covar(j,k);
	/*
			_covar(0,1) = covar(1,0);
			_covar(0,2) = covar(2,0);
			_covar(1,2) = covar(2,1);
	*/
		}

		//template<class POINT_LIST, class WEIGHT_LIST>
		//Covariance3D<T> doInitialize(const std::vector<Vector3D<T> >& points, const std::vector<double>& weights);

	private:
		boost::numeric::ublas::matrix<T> _covar;
	};

}
}

#endif /* COVARIANCE3D_HPP_ */
