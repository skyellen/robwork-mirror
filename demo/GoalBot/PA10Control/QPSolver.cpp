#include "QPSolver.hpp"
#include <cmath>
#include <math/MatrixMath.hpp>
#include <math/VectorMath.hpp>

using namespace boost::numeric::ublas;
using namespace rw::math;


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
vector<double> QPSolver::InequalitySolve(const matrix<double>& G, const vector<double>& d, const matrix<double>& A, const vector<double>& b) {
    const size_t n = G.size1();
    const size_t m = A.size1();
    
    vector<double> x(n);
    
    IntList Wk;
    IntList notWk;
    //We initialize Wk to contain all the active constraints	
    
    for (size_t i = 0; i<m; i++)
	if (b[i]==0) 
	    Wk.push_back(i);
	else
	    notWk.push_back(i);

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

	vector<double> g_k = prod(G,x)+d;
	for (size_t i = 0; i<g_k.size(); i++)
	    rhs(i) = -g_k(i);
	


	matrix<double> Minv = PseudoInverse(M);
	vector<double> pl(prod(Minv, rhs));

	/*	std::cout<<"M = "<<M<<std::endl;
	std::cout<<"rhs = "<<rhs<<std::endl;
	std::cout<<"pl = "<<pl<<std::endl;
	char ch[2]; std::cin.getline(ch, 1);*/
	vector<double> p_k(n);
	for (size_t i = 0; i<n; i++)
	    p_k(i) = pl(i);

	//Is the step zero
	if (norm_inf(p_k) < QP_EPSILON) {
	    vector<double> lambda(pl.size()-n);
	    for (size_t i = 0; i<lambda.size(); i++) {
		lambda(i) = pl(i+n);
	    }
	    if (min(lambda)>-QP_EPSILON) { 
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
	    double alpha = std::min(1.0, minval);
	    x = x+(p_k*alpha);
	    if (blocked) {
		Wk.push_back(*blockingConstraint);
		notWk.erase(blockingConstraint);
	    }
	}
    }
    std::cout<<"Did not terminate properly :-( \n";
    return x;
    
}
