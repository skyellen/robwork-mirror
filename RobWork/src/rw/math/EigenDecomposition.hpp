/*
 * EigenDecomposition.hpp
 *
 *  Created on: 11/01/2011
 *      Author: jimali
 */

#ifndef EIGENDECOMPOSITION_HPP_
#define EIGENDECOMPOSITION_HPP_

#include <Eigen/Eigen>

#include <vector>

namespace rw {
namespace math {

    template<class T=double>
    struct EigenDecomposition {
        EigenDecomposition(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> vectors,
                           Eigen::Matrix<T, Eigen::Dynamic, 1> values):
            _vectors(vectors),
            _values(values)
        {

        }

        /**
         * @brief returns all eigenvectors as columns in a matrix
         */
        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& getEigenVectors(){
            return _vectors;
        }

        /**
         * @brief returns the i'th eigenvector
         */
        Eigen::Matrix<T, Eigen::Dynamic, 1> getEigenVector(size_t i){
			return _vectors.col(i);			
            //boost::numeric::ublas::vector<T> v =
            //		boost::numeric::ublas::matrix_column<boost::numeric::ublas::matrix<T> >(_vectors, i);
            //return v;
        }

        /**
         * @brief return all eigenvalues
         */
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& getEigenValues() {			
            return _values;
        }

        /**
         * @brief returns the i'th eigenvalue
         */
        T getEigenValue(size_t i){
            return _values(i);
        }

        struct MapSort
        {
        public:
            Eigen::Matrix<T, Eigen::Dynamic, 1> &_values;
            std::vector<int>& _map;

            MapSort(Eigen::Matrix<T, Eigen::Dynamic, 1> &values,
                    std::vector<int>& map):
                _values(values),
                _map(map)
            {}

            bool operator()(const int& i0, const int& i1) {
                using namespace rw::math;
                return _values(i0) < _values(i1);
            }
        };

        /**
         * @brief sorts the eigen vectors according to their eigen value. The vector with smallest
         * eigen value has index 0
         */
        void sort(){
            std::vector<int> map(_values.size());
            for(size_t i=0;i<map.size();i++) map[i] = (int)i;

            std::sort( map.begin(), map.end(), MapSort(_values, map));
            // now the mapping determines how the new vectors are to be layed out
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> vectors = _vectors;
			Eigen::Matrix<T, Eigen::Dynamic, 1> values = _values;
            for(size_t i=0;i<map.size();i++) {
				_vectors.col(i) = vectors.col(map[i]);
				_values(i) = values(map[i]);
            	//ublas::matrix_column<ublas::matrix<T> >(_vectors,i) = ublas::matrix_column<ublas::matrix<T> >(vectors, map[i]);
             //   _values(i) = values(map[i]);
            }
        }

		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _vectors;
        Eigen::Matrix<T, Eigen::Dynamic, 1> _values;
    };
}
}

#endif /* EIGENDECOMPOSITION_HPP_ */
