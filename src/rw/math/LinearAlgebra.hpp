/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rw_math_LinearAlgebra_HPP
#define rw_math_LinearAlgebra_HPP

/**
 * @file LinearAlgebra.hpp
 */


#include "Vector3D.hpp"

#include <rw/common/macros.hpp>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <boost/numeric/bindings/lapack/workspace.hpp>
#include <boost/numeric/bindings/lapack/syev.hpp>
#include <boost/numeric/bindings/lapack/geev.hpp>

#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/ublas_vector.hpp>
#include <boost/numeric/bindings/lapack/gesvd.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{*/

    /**
     * @brief Collection of Linear Algebra functions
     */
    class LinearAlgebra
    {
    public:
        //! The standard Boost ublas matrix type.
        typedef boost::numeric::ublas::matrix<double> Matrix;

        //! A boost vector of complex numbers.
        typedef boost::numeric::ublas::vector<std::complex<double> > ComplexVector;

        /**
         * @brief Performs a singular value decomposition (SVD)
         *
         * The SVD computes the decomposition  
         * \f$ \mathbf{M}=\mathbf{U}*\mathbf{DiagonalMatrix(\sigma)}*\mathbf{V}^T \f$ .
         *
         * @param M [in] the matrix to decomposite
         * @param U [out] Result matrix \f$\mathbf{U}\f$
         * @param sigma [out] The \f$\mathbf{sigma}\f$ vector with diagonal elements
         * @param V [out] Result matrix \f$\mathbf{V}\f$
         */
        static void SVD(
            const Matrix& M,
            Matrix& U,
            boost::numeric::ublas::vector<double>& sigma,
            Matrix& V);

        /**
         * \brief Calculates the moore-penrose (pseudo) inverse of a matrix @f$ \mathbf{M}^+@f$
         *
         * \param am [in] the matrix @f$ \mathbf{M} @f$ to be inverted
         *
         * \param precision [in] the precision to use, values below this
         * treshold are considered singular
         *
         * \return the pseudo-inverse @f$ \mathbf{M}^+@f$ of @f$ \mathbf{M} @f$
         *
         * \author Lars Sønderby Jessen (<ljessen@mip.sdu.dk>)
         *
         * \f$ \mathbf{M}^+=\mathbf{V}\mathbf{\Sigma} ^+\mathbf{U}^T \f$ where
         * \f$ \mathbf{V} \f$, \f$ \mathbf{\Sigma} \f$ and \f$ \mathbf{U} \f$
         * are optained using Singular Value Decomposition (SVD)
         *
         * This method uses gesvd from LAPACK to perform SVD
         *
         */
        static Matrix PseudoInverse(const Matrix& am,
                                    double precision=1e-6);

        /**
         * @brief Checks the penrose conditions
         * @param A [in] a matrix
         * @param X [in] a pseudoinverse of A
         * @param prec [in] the tolerance
         *
         * @return true if the pseudoinverse X of A fullfills the penrose
         * conditions, false otherwise
         *
         * Checks the penrose conditions:
         *
         * @f$
         * AXA = A
         * @f$
         *
         * @f$
         * XAX = X
         * @f$
         *
         * @f$
         * (AX)^T = AX
         * @f$
         *
         * @f$
         * (XA)^T = XA
         * @f$
         */
        static bool CheckPenroseConditions(
            const Matrix& A,
            const Matrix& X,
            double prec = 1e-6);

        /**
         * \brief Calculates matrix determinant
         * \param m [in] a square matrix
         * \return the matrix determinant
         */
        template<class R>
        static inline double Det(const boost::numeric::ublas::matrix_expression<R>& m) 
        {
            assert(m().size1() == m().size2());

            boost::numeric::ublas::permutation_matrix<std::size_t> pivots(m().size1());

            // create copy
            boost::numeric::ublas::matrix<typename R::value_type> mLu(m);

            lu_factorize(mLu, pivots);

            double det = 1.0;

            for (std::size_t i=0; i < pivots.size(); ++i) {
                if (pivots(i) != i)
                    det *= -1.0;
                det *= mLu(i,i);
            }
            return det;
        }

        /**
         * @brief Calculates matrix inverse using lu_factorize and lu_substitute
         * @param M [in] input matrix @f$ \mathbf{M} @f$ to invert
         * @param Minv [out] output matrix @f$ \mathbf{M}^{-1} @f$
         **/
        template<class T>
        static void InvertMatrix (
            const boost::numeric::ublas::matrix_expression<T>& M,
            boost::numeric::ublas::matrix<typename T::value_type>& Minv)
        {
            using namespace boost::numeric::ublas;
            typedef permutation_matrix<std::size_t> pmatrix;

            // create a working copy of the input
            matrix<typename T::value_type> A(M);
            // create a permutation matrix for the LU-factorization
            pmatrix pm(A.size1());

            // perform LU-factorization
            lu_factorize(A,pm);

            // create identity matrix of "inverse"
            Minv.assign(identity_matrix<typename T::value_type>(A.size1()));

            // backsubstitute to get the inverse
            lu_substitute(A, pm, Minv);
        }

        /**
         * @brief Checks if a given matrix is in SO(n) (special orthogonal)
         * @param M [in] \f$ \mathbf{M} \f$
         * @return true if \f$ M\in SO(n) \f$
         *
         * \f$ SO(n) = {\mathbf{R}\in \mathbb{R}^{n\times n} :
         * \mathbf{R}\mathbf{R}^T=\mathbf{I}, det \mathbf{R}=+1} \f$
         *
         */
        template<class R>
        static inline bool IsSO(const boost::numeric::ublas::matrix_expression<R>& M)
        {
            return IsProperOrthonormal(M) && M().size1() == M().size2();
        }

        /**
         * @brief Checks if a given matrix is in so(n)
         * @param M [in] \f$ \mathbf{M} \f$
         * @return true if \f$ M\in so(n) \f$
         *
         * \f$ so(n) = {\mathbf{S}\in \mathbb{R}^{n\times n}:\mathbf{S}^T = -\mathbf{S}} \f$
         */
        template<class R>
        static inline bool Isso(const boost::numeric::ublas::matrix_expression<R>& M)
        {
            return IsSkewSymmetric(M) && M().size1() == M().size2();
        }

        /**
         * @brief Checks if a given matrix is skew-symmetrical
         * @param M [in] \f$ \mathbf{M} \f$ the matrix to check
         * @return true if the property \f$ \mathbf{M}=-\mathbf{M}^T \f$ holds, false otherwise
         *
         */
        template<class R>
        static inline bool IsSkewSymmetric(
            const boost::numeric::ublas::matrix_expression<R>& M)
        {
            return norm_inf(M+trans(M))==0.0;
        }

        /**
         * @brief Checks if a given matrix is proper orthonormal
         * @return true if the matrix is proper orthonormal, false otherwise
         *
         * A matrix is proper orthonormal if it is orthonormal and its determinant
         * is equal to \f$ +1 \f$
         */
        template<class R>
        static inline bool IsProperOrthonormal(
            const boost::numeric::ublas::matrix_expression<R>& r)
        {
            return IsOrthonormal(r) && Det(r) == 1.0;
        }

        /**
         * @brief Checks if a given matrix is orthonormal
         * @return true if the matrix is orthonormal, false otherwise
         *
         * A matrix is orthonormal if all of it's column's are mutually orthogonal
         * and all of it's column's has unit length.
         *
         * that is for any \f$ i, j \f$ the following holds
         * \f$ col_i . col_j = 0 \f$ and \f$ ||col_i|| = 1 \f$
         *
         * Another nessesary and sufficient condition of orthonormal matrices is that
         * \f$ \mathbf{M}\mathbf{M}^T=I \f$
         */
        template<class R>
        static inline bool IsOrthonormal(
            const boost::numeric::ublas::matrix_expression<R>& r)
        {
            return norm_inf(
                prod(r, trans(r)) -
                boost::numeric::ublas::identity_matrix<typename R::value_type>(
                    r().size2(), r().size2())
                ) == 0.0;
        }

        /**
         * @brief Computes the eigenvalue decomposition of a symmetric matrix
         *
         * Given a symmetric matrix \f$ \mathbf{A} \in \mathbb{R}^n \f$ the
         * eigenvalue decomposition giving \f$\lambda_i, \mathbf{x}_i \f$ such
         * that \f$\lambda_i \mathbf{x}_i=\mathbf{A}\mathbf{x}_i \f$.
         *
         * @param A [in] the matrix \f$\mathbf{A}\f$
         *
         * @return std::pair in which the eigenvectors and eigenvectors are
         * stored as columns in the matrix and elements in the vector
         * respectively.
         */
        template<class T>
        static std::pair<boost::numeric::ublas::matrix<T>, boost::numeric::ublas::vector<T> >
        EigenDecompositionSymmetric(boost::numeric::ublas::matrix<T>& A)
        {
            typedef boost::numeric::ublas::matrix<T, boost::numeric::ublas::column_major>
                TColumnMatrix;
            typedef boost::numeric::ublas::zero_vector<T> TZeroVector;
            typedef boost::numeric::ublas::vector<T> TVector;
            typedef boost::numeric::ublas::matrix<T> TMatrix;

            using namespace boost::numeric::bindings::lapack;

            assert(A.size1() == A.size2());
            size_t n = A.size1();
            TColumnMatrix Ac = A;

            TZeroVector Wz(n);
            TVector Wc(Wz);

            syev<TColumnMatrix, TVector >('V', 'U', Ac, Wc, optimal_workspace());

            return std::make_pair(TMatrix(Ac), Wc);
        }

        /**
         * @brief Computes the eigenvalue decomposition
         *
         * Given a matrix \f$ \mathbf{A} \in \mathbb{R}^n \f$ the eigenvalue
         * decomposition giving \f$\lambda_i, \mathbf{x}_i \f$ such that
         * \f$\lambda_i \mathbf{x}_i=\mathbf{A}\mathbf{x}_i \f$.
         *
         * @param A [in] the matrix \f$\mathbf{A}\f$
         *
         * @return std::pair in which the eigenvectors and eigenvectors are
         * stored as columns in the matrix and elements in the vector
         * respectively.
         */
        template<class T>
        static std::pair<Matrix, ComplexVector>
        EigenDecomposition(boost::numeric::ublas::matrix<T>& A)
        {
            typedef boost::numeric::ublas::matrix<double, boost::numeric::ublas::column_major>
                ColumnMatrix;

            using namespace boost::numeric::bindings::lapack;

            assert(A.size1() == A.size2());

            size_t n = A.size1();
            ColumnMatrix Ac(A);

            optimal_workspace workspace;

            ComplexVector Wc(n);

            ColumnMatrix Vr(n,n);
            ColumnMatrix Vl(n,n);

            geev<ColumnMatrix, ComplexVector, ColumnMatrix >(Ac, Wc, &Vl, &Vr, workspace);

            std::cout<<"Wc = "<<Wc<<std::endl;
            std::cout<<"A = "<<Ac<<std::endl;
            std::cout<<"Vl = "<<Vl<<std::endl;
            std::cout<<"Vr = "<<Vr<<std::endl;

            return std::make_pair(Matrix(Vr), Wc);
        }

        /**
         * @brief Returns vector with the absolute values
         *
         * Given a vector \f$v=[v_1,v_2,\ldots,v_n]\f$ then Abs(v) is defined as
         * \f$Abs(v)=[abs(v_1),abs(v_i),\ldots,abs(v_n)] \f$
         *
         * @param v [in] the vector \f$v\f$
         * @return the vector \f$Abs(v)\f$
         */
        template<class T>
        static boost::numeric::ublas::vector<T> Abs(
            const boost::numeric::ublas::vector<T>& v)
        {
            RW_DEPRECATED("Deprecated.  Use Math::Abs");
            boost::numeric::ublas::vector<T> result(v.size());
            for (size_t i = 0; i<v.size(); i++)
                result[i] = std::fabs(v[i]);
            return result;
        }
        
        /**
         * @brief Returns the smallest element of v
         *
         * If the vector has zero length, the method returns 0
         *
         * @param v [in] the vector v
         * @return the smallest element
         */
        template<class T>
        static T Min(const boost::numeric::ublas::vector<T>& v)
        {
            RW_DEPRECATED("Deprecated. Use Math::Min()");
            if (v.size() == 0)
                return 0;
            T minval = v(0);
            for (size_t i = 1; i<v.size(); i++)
                if (v(i)<minval)
                    minval = v(i);
            return minval;
        }

        /**
         * @brief Returns the largest element of v
         *
         * If the vector has zero length, the method returns 0
         *
         * @param v [in] the vector v
         * @return the largest element
         */
        template<class T>
        static T Max(const boost::numeric::ublas::vector<T>& v)
        {
            RW_DEPRECATED("Deprecated. Use Math::Max");
            if (v.size() == 0)
                return 0;
            T maxval = v(0);
            for (size_t i = 1; i<v.size(); i++)
                if (v(i)>maxval)
                    maxval = v(i);
            return maxval;
        }   
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
