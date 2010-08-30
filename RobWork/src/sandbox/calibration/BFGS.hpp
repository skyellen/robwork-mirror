#ifndef BFGS_HPP_
#define BFGS_HPP_

/**
 * @file BFGS.hpp
 */

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/operation.hpp>

namespace rwlibs { namespace algorithms {

	/** @addtogroup algorithms */
		/*@{*/

		/**
		 * @brief BFGS is a class including the BFGS minimization algorithm.
		 *
		 * The BFGS minimization algorithm has been implemented as described in "Numerical Optimization - by Jorge Nocedal and Stephen J. Wright" chapter 6+3.
		 * \sa \ref bfgsExample.cpp "example/bfgsExample.cpp"
		 */
	class BFGS {

		public:
			//! Vector type used in the minimazation algorithm.
        		typedef boost::numeric::ublas::vector<double> vector;
			//! MAtrix type used in the minimazation algorithm.
        		typedef boost::numeric::ublas::matrix<double> matrix;

			/**
			 * @brief Minimisation function struct.
			 */
			struct BFGS_function_struct
			{
				/** Function pointer to the static minimization function @f$f(vec{x})@f$. */
				double (* f) (const vector * x, void * params);
				/** Function pointer to the static minimization function @f$df(vec{x})@f$. */
				void (* df) (const vector * x, void * params, vector * g);
				/** Void pointer to optional data that the minimization function might require */
				void * params;
			};

			/**
			 * @brief Optimization status.
			 */
			enum OPTM_STATUS {
				/** Indicating a problem with the numerical precision when evaluating the gradient. */
				GRADIENTWARNING=0,
				/** Indicating a successfully minimization. */
				SUCCESS};

			/**
			 * @brief Minimize a function using the BFGS algorithm.
			 * @param startguess Start guess for the minimizer parameters. Replaced with minima solution at end of minimization.
			 * @param function BFGS_function_struct including pointers to the minimization function f, df and a void pointer to other data for the minimization function.
			 * @param tolerance Indicating when an acceptable minima has been found by evaluating if @f$ tolerance>||\Delta f(x)||_2 @f$.
			 * @param iterationLimit Maximum number of iterations for the BFGS algorithm.
			 * @param initialStepsize Initial step size for the BFGS algorithm.
			 * @param c1 Value used to ensure the "strong Wolfe conditions" are satisfied with the value c1. See "Numerical Optimization - by Jorge Nocedal and Stephen J. Wright" chapter 3. Typical value = 1e-4.
			 * @param c2 Value used to ensure the "strong Wolfe conditions" are satisfied with the value c2. See "Numerical Optimization - by Jorge Nocedal and Stephen J. Wright" chapter 3. Typical value = 0.9.
			 * @param alphamax Maximum stepsize used in iterations. Typical value of 1.0 is used to produce superlinear convergence of the overall algorithm.
			 * @return GRADIENTWARNING on numerically precision problems SUCCESS when a minima is found.
			 */
			static int optimizer(
					vector &startguess,
					BFGS_function_struct function,
					double tolerance,
					unsigned int iterationLimit,
					double initialStepsize = 1.0,
					double c1 = 1e-4,
					double c2 = 0.9,
					double alphamax = 1.0);



		private:
			BFGS() {}

			static void colDotRow(
					vector &colvec,
					vector &rowvec,
					matrix &result);

			static double lineSearch(
					BFGS_function_struct function,
					vector &xk,
					vector &pk,
					double c1,
					double c2,
					double alphamax);

			static double phiGradient(
					BFGS_function_struct function,
					vector &xk,
					vector &pk,
					double alpha,
					vector &tempArray,
					double phi_alpha,
					double eps);

			static double zoom(
					double &alphalow,
					double &alphahigh,
					double &phi_alphalow,
					double &dphi_alphalow,
					double &phi_alphahigh,
					BFGS_function_struct function,
					vector &xk,
					vector &pk,
					vector &tempArray,
					double phi_alpha_zero,
					double dphi_alpha_zero,
					double c1,
					double c2,
					double eps);

			static double quadraticInterpolation(
					double phi_alpha_lo,
					double dphi_alpha_lo,
					double alpha_lo,
					double phi_alpha_hi,
					double alpha_hi);
	};
	/** \example bfgsExample.cpp
	 * Example of using the BFGS optimization algorithm for finding a minimum in the Rosenbrock function.
	 *
	 * The Rosenbrock function are defined by: @f$ f(x, y) = (1-x)^2 + 100(y-x^2)^2 @f$
	 */

/*@}*/
} }
#endif /* BFGS_HPP_ */
