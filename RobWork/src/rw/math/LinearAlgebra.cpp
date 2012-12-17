/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#include "LinearAlgebra.hpp"

#include "boost/numeric/ublas/matrix_proxy.hpp"

#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/ublas_vector.hpp>
#include <boost/numeric/bindings/lapack/gesvd.hpp>

using namespace rw::math;

namespace lapack = boost::numeric::bindings::lapack;
using namespace boost::numeric::ublas;

typedef LinearAlgebra::BoostMatrix<double>::type BoostMatrixType;
typedef zero_matrix<double> BoostZeroMatrixType;
typedef matrix<double, column_major> BoostColumnMatrixType;
typedef matrix_range<BoostColumnMatrixType> BoostColumnMatrixTypeRange;


extern "C" {

    //void sptsv_(int *N, int *NRHS, float *D, float *e, float *b, int *ldb, long *info);

    // same as above but using double precision instead
    //void dptsv_(int *n, int *nrhs, double *d, double *e, double *b, int *ldb, long *info);

}

#ifdef RW_USE_UBLAS_LAPACK

void LinearAlgebra::svd(
    const BoostMatrixType& M,
    BoostMatrixType& U,
	BoostVector<double>::type& sigma,
    BoostMatrixType& V)
{
    // rows
    const size_t m = M.size1();

    // columns
    const size_t n = M.size2();

    // Define the U, Sigma and V^t
    BoostColumnMatrixType u(m, n);
    vector<double> s(n);
    BoostColumnMatrixType vt(n, n);

    BoostColumnMatrixType Mc(M);
    // Calculate Singular Value Decomposition of A
    lapack::gesvd(Mc, s, u, vt);

    U = u;
    sigma = s;
    V = trans(vt);
}

#endif

#define RW_USE_UBLAS_LAPACK	
#ifdef RW_USE_UBLAS_LAPACK

BoostMatrixType LinearAlgebra::pseudoInverse(const BoostMatrixType& am, double precision)
{
    // rows
    const size_t m = am.size1();

    // columns
    const size_t n = am.size2();

    // If matrix is empty return an empty matrix
    if (m == 0 || n == 0)
        return BoostMatrixType();

    if (n > m) 
		return trans(pseudoInverse(trans(am), precision));

    // convert am to column_major form
    BoostColumnMatrixType a(am);

    // Define the U, Sigma and V^t
    BoostColumnMatrixType u(m, n);
    vector<double> s(n);
    BoostColumnMatrixType vt(n, n);

    // Calculate Singular Value Decomposition of A
    lapack::gesvd(a, s, u, vt);

    // Find rank of A (number of non-zero diagonal elements in Sigma)
    size_t rank = 0;
    while (rank < n && s[rank] >= precision) {
        rank++;
    }

    if (rank == 0) return BoostZeroMatrixType(n, m);

    BoostMatrixType s_ = BoostZeroMatrixType(rank, rank);
    for (size_t count = 0; count < rank; count++)
        s_(count, count) = 1.0 / s(count);

    // Calculate: V * S * Ut
    BoostMatrixType t1(prod(trans(BoostColumnMatrixTypeRange(vt,range(0, rank),range(0, n))), s_));

    return prod(t1, trans(BoostColumnMatrixTypeRange(u, range(0, m), range(0, rank))));
}

#endif

Eigen::MatrixXd LinearAlgebra::pseudoInverseEigen(const Eigen::MatrixXd& am, double precision) {
	const Eigen::JacobiSVD<Eigen::MatrixXd> svd = am.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sigma= svd.singularValues();
	for ( long i=0; i<sigma.cols(); ++i) {
		if ( sigma(i) > precision )
			sigma(i)=1.0/sigma(i);
		else 
			sigma(i)=0;
	}

	return svd.matrixV() * sigma.asDiagonal()* svd.matrixU().transpose();

}

bool LinearAlgebra::checkPenroseConditions(
    const BoostMatrixType& A,
    const BoostMatrixType& X,
    double prec)
{
    BoostMatrixType AX = prod(A, X);
    BoostMatrixType XA = prod(X, A);

    if (norm_inf(prod(AX, A) - A) > prec)
        return false;
    if (norm_inf(prod(XA, X) - X) > prec)
        return false;
    if (norm_inf(trans(AX) - AX) > prec)
        return false;
    if (norm_inf(trans(XA) - XA) > prec)
        return false;

    return true;
}

bool LinearAlgebra::checkPenroseConditions(
	const Eigen::MatrixXd& A,
	const Eigen::MatrixXd& X,
    double prec)
{
	const Eigen::MatrixXd AX = A*X;
	const Eigen::MatrixXd XA = X*A;

	if (((AX*A)-A).lpNorm<Eigen::Infinity>() > prec)
		return false;
	if (((XA*X)-X).lpNorm<Eigen::Infinity>() > prec)
		return false;
	if ((AX.transpose() - AX).lpNorm<Eigen::Infinity>() > prec)
		return false;
	if ((XA.transpose() - XA).lpNorm<Eigen::Infinity>() > prec)
		return false;
	return true;
}


void LinearAlgebra::lapack_triDiagonalSolve(int *N, int *NRHS, float *D, float *e, float *b, int *ldb, int *info){
    LAPACK_SPTSV(N, NRHS, D, e, b, ldb, info);
}

void LinearAlgebra::lapack_triDiagonalSolve(int *N, int *NRHS, double *D, double *e, double *b, int *ldb, int *info){
    LAPACK_DPTSV(N,NRHS,D,e,b,ldb,info);
}

