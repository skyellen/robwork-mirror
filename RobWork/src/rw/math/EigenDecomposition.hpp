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
	//! @brief Type representing a set of eigen values and eigen vectors.
    template<class T=double>
    struct EigenDecomposition {
    	/**
    	 * @brief Construct new decomposition.
    	 * @param vectors [in] the eigen vectors as columns in a matrix.
    	 * @param values [in] the corresponding eigen values.
    	 */
        EigenDecomposition(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> vectors,
                           Eigen::Matrix<T, Eigen::Dynamic, 1> values):
            _vectors(vectors),
            _values(values)
        {

        }

        /**
         * @brief returns all eigenvectors as columns in a matrix
         * @return reference to the matrix.
         */
        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& getEigenVectors(){
            return _vectors;
        }

        /**
         * @brief returns the i'th eigenvector
         * @return the eigen vector.
         */
        Eigen::Matrix<T, Eigen::Dynamic, 1> getEigenVector(size_t i){
			return _vectors.col(i);			
            //boost::numeric::ublas::vector<T> v =
            //		boost::numeric::ublas::matrix_column<boost::numeric::ublas::matrix<T> >(_vectors, i);
            //return v;
        }

        /**
         * @brief return all eigenvalues
         * @return the eigen values.
         */
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& getEigenValues() {			
            return _values;
        }

        /**
         * @brief returns the i'th eigenvalue
         * @return the eigenvalue.
         */
        T getEigenValue(size_t i){
            return _values(i);
        }

        //! @brief Sort function for ordering of eigen values and vectors.
        struct MapSort
        {
        private:
            Eigen::Matrix<T, Eigen::Dynamic, 1> &_values;

        public:
            /**
             * @brief Construct new sort function struct.
             * @param values [in] the eigen values.
             */
            MapSort(Eigen::Matrix<T, Eigen::Dynamic, 1> &values):
                _values(values)
            {}

            /**
             * @brief Compare the eigen values with the given indices.
             * @param i0 [in] index of first eigenvalue.
             * @param i1 [in] index of second eigenvalue.
             * @return true if first eigenvalue comes before the second (it is smaller), false otherwise.
             */
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

            std::sort( map.begin(), map.end(), MapSort(_values));
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

    private:
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _vectors;
        Eigen::Matrix<T, Eigen::Dynamic, 1> _values;
    };
}
}

#endif /* EIGENDECOMPOSITION_HPP_ */
