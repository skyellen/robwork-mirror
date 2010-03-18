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

typedef LinearAlgebra::Matrix<double>::type MatrixType;
typedef zero_matrix<double> ZeroMatrixType;
typedef matrix<double, column_major> ColumnMatrixType;
typedef matrix_range<ColumnMatrixType> ColumnMatrixTypeRange;

void LinearAlgebra::svd(
    const MatrixType& M,
    MatrixType& U,
    vector<double>& sigma,
    MatrixType& V)
{
    // rows
    const size_t m = M.size1();

    // columns
    const size_t n = M.size2();

    // Define the U, Sigma and V^t
    ColumnMatrixType u(m, n);
    vector<double> s(n);
    ColumnMatrixType vt(n, n);

    ColumnMatrixType Mc(M);
    // Calculate Singular Value Decomposition of A
    lapack::gesvd(Mc, s, u, vt);

    U = u;
    sigma = s;
    V = trans(vt);
}

MatrixType LinearAlgebra::pseudoInverse(const MatrixType& am, double precision)
{
    // rows
    const size_t m = am.size1();

    // columns
    const size_t n = am.size2();

    // If matrix is empty return an empty matrix
    if (m == 0 || n == 0)
        return MatrixType();

    if (n > m) return trans(pseudoInverse(trans(am), precision));

    // convert am to column_major form
    ColumnMatrixType a(am);

    // Define the U, Sigma and V^t
    ColumnMatrixType u(m, n);
    vector<double> s(n);
    ColumnMatrixType vt(n, n);

    // Calculate Singular Value Decomposition of A
    lapack::gesvd(a, s, u, vt);

    // Find rank of A (number of non-zero diagonal elements in Sigma)
    size_t rank = 0;
    while (rank < n && s[rank] >= precision) {
        rank++;
    }

    if (rank == 0) return ZeroMatrixType(n, m);

    MatrixType s_ = ZeroMatrixType(rank, rank);
    for (size_t count = 0; count < rank; count++)
        s_(count, count) = 1.0 / s(count);

    // Calculate: V * S * Ut
    MatrixType t1(prod(trans(ColumnMatrixTypeRange(vt,range(0, rank),range(0, n))), s_));

    return prod(t1, trans(ColumnMatrixTypeRange(u, range(0, m), range(0, rank))));
}

bool LinearAlgebra::checkPenroseConditions(
    const MatrixType& A,
    const MatrixType& X,
    double prec)
{
    MatrixType AX = prod(A, X);
    MatrixType XA = prod(X, A);

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
