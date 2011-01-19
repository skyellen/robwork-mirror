/*
 * EigenDecomposition.hpp
 *
 *  Created on: 11/01/2011
 *      Author: jimali
 */

#ifndef EIGENDECOMPOSITION_HPP_
#define EIGENDECOMPOSITION_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <boost/numeric/ublas/matrix.hpp>


namespace rw {
namespace math {

    template<class T=double>
    struct EigenDecomposition {
        EigenDecomposition(boost::numeric::ublas::matrix<T> vectors,
                           boost::numeric::ublas::vector<T> values):
            _vectors(vectors),
            _values(values)
        {

        }

        /**
         * @brief returns all eigenvectors as columns in a matrix
         */
        const boost::numeric::ublas::matrix<T>& getEigenVectors(){
            return _vectors;
        }

        /**
         * @brief returns the i'th eigenvector
         */
        boost::numeric::ublas::vector<T> getEigenVector(size_t i){
            using namespace boost::numeric::ublas;
            vector<T> v = matrix_column<matrix<T> >(_vectors, i);
            return v;
        }

        /**
         * @brief return all eigenvalues
         */
        const boost::numeric::ublas::vector<T>& getEigenValues(){
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
            boost::numeric::ublas::vector<T> &_values;
            std::vector<int>& _map;

            MapSort(
                    boost::numeric::ublas::vector<T> &values,
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

            using namespace boost::numeric::ublas;
            std::vector<int> map(_values.size());
            for(size_t i=0;i<map.size();i++) map[i] = i;

            std::sort( map.begin(), map.end(), MapSort(_values, map));
            // now the mapping determines how the new vectors are to be layed out
            matrix<T> vectors = _vectors;
            vector<T> values = _values;
            for(size_t i=0;i<map.size();i++){
                matrix_column<matrix<T> >(_vectors,i) = matrix_column<matrix<T> >(vectors, map[i]);
                _values(i) = values(map[i]);
            }
        }

        boost::numeric::ublas::matrix<T> _vectors;
        boost::numeric::ublas::vector<T> _values;
    };
}
}

#endif /* EIGENDECOMPOSITION_HPP_ */
