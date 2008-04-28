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

#ifndef rwlibs_algorithms_qpcontroller_QPSolver_HPP
#define rwlibs_algorithms_qpcontroller_QPSolver_HPP

/**
 * @file QPSolver.hpp
 */

#include <list>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace rwlibs { namespace algorithms { namespace qpcontroller {


    /**
     * @brief Class providing an algorithms for solving the quadratic optimization
     * problem associated with the QPController 
     */
    class QPSolver  
    {
    public:
        enum Status {
            SUCCESS = 0, /** Solved */
            SUBOPTIMAL, /** Constraint satisfied but the result may be suboptimal. This may occurs
                              due to round off errors */
            ERROR /** Could not find a solution statisfying all the constraints */        
        };
        
        /**
         * Solves the quadratic problem of minimizing 1/2 x^T.G.x+d^T.x subject
         * to A.x>=b The method used is an iterative method from "Numerical
         * Optimization" by Jorge Nocedal and Stephen J. Wright, Springer 1999.
         * In this implementation we'll require that b<= s.t. x=0 is a feasible
         * initial value
         *
         * \param G [in] The G matrix. It is required that G is n times n and is
         * positive semidefinite
         *
         * \param d [in] Vector of length n
         *
         * \param A [in] Matrix used to represent the linear inequality constraints.
         * The dimensions should be m times n
         *
         * \param b [in] Vector with the lower limit for the constraints. We'll
         * assume that b<=0. The length of b should be m
         * 
         * \param xstart [in] Default start configuration of the iterative algorithm
         * 
         * \param status [out] Gives the status of the solving
         */
        static boost::numeric::ublas::vector<double>
        inequalitySolve(
            const boost::numeric::ublas::matrix<double>& G, 
            const boost::numeric::ublas::vector<double>& d, 
            boost::numeric::ublas::matrix<double>& A, 
            const boost::numeric::ublas::vector<double>& b,
            const boost::numeric::ublas::vector<double>& xstart,
            Status& status);
        
        //TODO Investigate the possibility of making a hot-start of the
        //algorithm for better performance
        
    private:
        static boost::numeric::ublas::vector<double> getInitialConfig(
            boost::numeric::ublas::matrix<double>& A, 
            const boost::numeric::ublas::vector<double>& b);
        
        static boost::numeric::ublas::vector<double> safeApprox(
            boost::numeric::ublas::matrix<double>& A, 
            const boost::numeric::ublas::vector<double>& b);
    };
    
}}} // end namespaces

#endif // end include guard
