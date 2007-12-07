// QPSolver.h: interface for the QPSolver class.
//
//////////////////////////////////////////////////////////////////////

#ifndef QPSOLVER_HPP
#define QPSOLVER_HPP


#include <list>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

typedef std::list<int> IntList;

const double QP_EPSILON = 1e-12;

class QPSolver  
{
public:
  /**
   * Solves the quadratic problem of minimizing 1/2 x^T.G.x+d^T.x subject to A.x>=b
   * The method used is an iterative method from "Numerical Optimization" by Jorge Nocedal
   * and Stephen J. Wright, Springer 1999.
   * In this implementation we'll require that b<= s.t. x=0 is a feasible initial value
   *
   * \param G The G matrix. It is required that G is n times n and is positive semidefinite
   * \param d Vector of length n
   * \param A Matrix used to represent the linear inequality constraints. The dimensions should be m times n
   * \param b Vector with the lower limit for the constraints. We'll assume that b<=0. The length of b should be m
   */
  static boost::numeric::ublas::vector<double> InequalitySolve(const boost::numeric::ublas::matrix<double>& G, 
							       const boost::numeric::ublas::vector<double>& d, 
							       const boost::numeric::ublas::matrix<double>& A, 
							       const boost::numeric::ublas::vector<double>& b);

    //TODO Investigate the possibility of making a hot-start of the algorithm for better performance
};

#endif // #ifndef QPSOLVER_HPP

