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


#include "QPSolver.hpp"

#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Math.hpp>
#include <rw/common/macros.hpp>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

#include <cmath>

using namespace boost::numeric::ublas;
using namespace rw::math;

using namespace rwlibs::algorithms::qpcontroller;


namespace {

    const double EPS = 1e-12;
    const double ERROR_LIMIT = 1e-10;

    typedef std::list<int> IntList;

    const double QP_EPSILON = 1e-12;
}


vector<double> QPSolver::getInitialConfig(matrix<double>& A, const vector<double>& b) {
    size_t m = A.size1();
    size_t n = A.size2();

    vector<double> x(n);


    for (size_t i = 0; i<n; i++) {
        x(i) = 0.5*(b(2*i)-b(2*i+1));
    }

    vector<double> ax;
    ax = prod(A, x);

    vector<double> delta;

    if (ax(m-1)+EPS<b(m-1)) {
        for (size_t i = 0; i<b.size()-1; i++) {
            matrix_row<matrix<double> > grad(A, i);
            delta = prod(A, grad);

            if (fabs(delta(m-1))>EPS) {
                double error = b(m-1)-ax(m-1);
                double dq1 = error/delta(m-1);

                if (dq1<0) {
                    double limit = (b(i)-ax(i));
                    double dq2 = limit/(delta(i));
                    x += grad*std::max(dq1, dq2);
                    ax = prod(A, x);
                }
            }
        }
    }
    return x;

}


vector<double> QPSolver::safeApprox(matrix<double>& A, const vector<double>& b) {

    vector<double> x = getInitialConfig(A, b);

    size_t m = A.size1();


    matrix_row<matrix<double> > grad(A, m-1);
    double alpha = (b(m-1)-inner_prod(grad, x))/inner_prod(grad, grad);
    x += alpha*grad;
    return x;
}

/**
 * Solves the quadratic problem of minimizing 1/2 x^T.G.x+d^T.x subject to A.x>=b
 * The method used is an iterative method from "Numerical Optimization" by Jorge Nocedal
 * and Stephen J. Wright, Springer 1999.
 * In this implementation we'll require that b<=0 s.t. x=0 is a feasible initial value
 *
 * \param G The G matrix. It is required that G is n times n and is positive semidefinite
 * \param d Vector of length n
 * \param A Matrix used to represent the linear inequality constraints. The dimensions should be m times n
 * \param b Vector with the lower limit for the constraints. We'll assume that b<=0. The length of b should be m
 */
vector<double> QPSolver::inequalitySolve(
    const matrix<double>& G,
    const vector<double>& d,
    matrix<double>& A,
    const vector<double>& b,
    const vector<double>& xstart,
    Status& status)
{
    const size_t n = G.size1();
    const size_t m = A.size1();

    //   bool finished = false;
    //size_t itcnt = 0;
    matrix_row<matrix<double> > jz(A, m-1);


    vector<double> x = xstart;//getInitialConfig(A, b);

    vector<double> bcompare = prod(A, x);
    for (size_t i = 0; i<b.size(); i++)
        if (bcompare(i)+ERROR_LIMIT < b(i)) {
            status = FAILURE;
            std::cout<<"Warning: Invalid start configuration"<<i<<"   "<<bcompare(i)-b(i)<<std::endl;
        //    return xstart;

        }


    IntList Wk;
    IntList notWk;

    //We initialize Wk to contain all the active constraints
    for (size_t i = 0; i<m; i++)
        if (fabs(b[i] - bcompare[i])<EPS)
            Wk.push_back((int)i);
        else
            notWk.push_back((int)i);

    for (size_t i = 0; i<n*(m+1); i++) {
        /*
         * Construct  |G   A_k^T | |p      |  |-g_k|
         *            |A_k 0     |.|\lambda|= | 0  |
         * where g_k=G.x_k+d
         */
        matrix<double> M = zero_matrix<double>(n+Wk.size());


        vector<double> rhs = zero_vector<double>(n+Wk.size());

        for (size_t i = 0; i<G.size1(); i++)
            for (size_t j = 0; j<G.size2(); j++)
                M(i,j) = G(i,j);
        size_t next = n;
        for (IntList::const_iterator ci = Wk.begin(); ci != Wk.end(); ci++) {
            for (size_t j = 0; j<n; j++) {
                M(next, j) = A(*ci, j);
                M(j, next) = -A(*ci, j);
            }
            ++next;
        }
        vector<double> g_k;
        g_k = prod(G,x)+d;
        for (size_t i = 0; i<g_k.size(); i++)
            rhs(i) = -g_k(i);


        matrix<double> Minv = LinearAlgebra::pseudoInverse(M, 1e-12);

        vector<double> pl(prod(Minv, rhs));

        vector<double> p_k(n);
        for (size_t i = 0; i<n; i++)
            p_k(i) = pl(i);

        //Is the step zero
        //      std::cout<<"p_k = "<<p_k<<std::endl;
        if (norm_inf(p_k) < 1e-8) {

            vector<double> lambda;
            lambda = (vector_range<vector<double> >(pl, range(n,pl.size())));

            /*      vector<double> lambda(pl.size()-n);
            for (size_t i = 0; i<lambda.size(); i++) {
                lambda(i) = pl(i+n);
                }*/

            // std::cout<<"lambda = "<<min(lambda)<<std::endl;

            if (Math::min(lambda) >= -QP_EPSILON) {
                vector<double> bcompare(x.size());
                bcompare = prod(A, x);
                for (size_t i = 0; i<b.size(); i++)

                    if (bcompare(i) + ERROR_LIMIT < b(i)) {

                        std::cout
                            <<"Warning: Could not find valid result for "
                            << i << " error = " << (b(i) - bcompare(i))
                            << std::endl;

                        std::cout << "Returns standard solution" << std::endl;

                        status = SUBOPTIMAL;
                        return xstart;
                    }
                status = SUCCESS;
                return x;
            }
            else {
                IntList::iterator min = Wk.begin();
                int next = 0;
                double minVal = lambda(next);
                for (IntList::iterator it = Wk.begin(); it != Wk.end(); ++it) {
                    if (lambda(next)<minVal) {
                        min = it;
                        minVal = lambda(next);
                    }
                    ++next;
                }
                notWk.push_back(*min);
                Wk.erase(min);
            }
        } else {
            //compute \alpha_k as min(1,min (b_i-a_i^T x_k)/(a_i^T p_k) with i\notin W_k, a_i^Tp_k<0

            double minval = 1;
            IntList::iterator blockingConstraint;
            bool blocked = false;
            for (IntList::iterator it = notWk.begin(); it != notWk.end(); it++) {
                double b_i = b[*it];
                vector<double> a_i(A.size2());
                for (size_t i = 0; i<A.size2(); i++)
                    a_i(i) = A(*it,i);
                double a_iTp_k = inner_prod(a_i,p_k);

                if (a_iTp_k < 0) {
                    double val = (b_i-inner_prod(a_i,x))/(a_iTp_k);

                    if (val<=minval) {
                        minval = val;
                        blockingConstraint = it;
                        blocked = true;
                    }
                }
            }
            //      std::cout<<"minval = "<<minval<<std::endl;
            //std::cout<<"blocked "<<blocked<<"  "<<true<<std::endl;
            double alpha = std::min(1.0, minval);
            x = x+(p_k*alpha);
            if (blocked) {
                Wk.push_back(*blockingConstraint);
                notWk.erase(blockingConstraint);
            }
        }
    }

    RW_WARN("QPSolver did not terminate correctly. This may be due to round off error.");


    bcompare = prod(A, x);

    for (size_t i = 0; i<b.size(); i++) {
        if (bcompare(i)+ERROR_LIMIT < b(i)) {
            std::cout<<"QPSolver Failed to find result valid for all constraints "<<i<<std::endl;
            RW_WARN("QPSolver failed to find a result valid for Constraint "<<i);
            status = FAILURE;
            return xstart;
        }
    }

    status = SUBOPTIMAL;


    return x;

}
