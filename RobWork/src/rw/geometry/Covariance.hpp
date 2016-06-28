/*
 * Covariance3D.hpp
 *
 *  Created on: 04/08/2010
 *      Author: jimali
 */

#ifndef COVARIANCE3D_HPP_
#define COVARIANCE3D_HPP_

#include <vector>

#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/Geometry.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/EigenDecomposition.hpp>


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

        Covariance(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& matrix):_covar(matrix)
        {
        }

		virtual ~Covariance(){};

		const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& getMatrix(){ return _covar;};

		rw::math::EigenDecomposition<T> eigenDecompose(){
			typedef std::pair<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<T, Eigen::Dynamic, 1> > ResultType;
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

			//const size_t nrOfPoints = points.size();

			_covar = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(DIM, DIM);
			Eigen::Matrix<T, Eigen::Dynamic, 1> centroid = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(DIM);

			// calculate centroid
			size_t nrOfPoints = 0;
			RandomAccessIterator first_tmp = first;
			for( ;first_tmp!=last; ++first_tmp){
                nrOfPoints++;
                for(size_t j=0; j<DIM; j++)
                    centroid[j] += (*first_tmp)[j];
			}

			// scale according to nr points
            centroid = centroid*((T)1.0/nrOfPoints);


			// next we compute the covariance elements
			Eigen::VectorXd p = Eigen::VectorXd::Zero(DIM);
			for( ;first!=last; ++first){
			    for(size_t k=0; k<DIM; k++){
			        p[k] = (*first)[k]-centroid[k];
			    }

				for(size_t j=0; j<DIM; j++){
					for(size_t k=j; k<DIM; k++){
					    _covar(k,j) += (*first)[k] * (*first)[j];
					}
				}
			}

			// fill in all covariance elements
			for(size_t j=1; j<DIM; j++)
				for(size_t k=0; k<j; k++)
					_covar(k,j) = _covar(j,k);
		}

		//template<class POINT_LIST, class WEIGHT_LIST>
		//Covariance3D<T> doInitialize(const std::vector<Vector3D<T> >& points, const std::vector<double>& weights);

	private:
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _covar;
	};

}
}

#endif /* COVARIANCE3D_HPP_ */
