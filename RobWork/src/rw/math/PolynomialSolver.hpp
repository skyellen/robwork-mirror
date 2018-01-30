/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_MATH_POLYNOMIALSOLVER_HPP_
#define RW_MATH_POLYNOMIALSOLVER_HPP_

/**
 * @file PolynomialSolver.hpp
 *
 * \copydoc rw::math::PolynomialSolver
 */

#include "Polynomial.hpp"

#include <vector>
#include <complex>

namespace rw {
namespace math {
//! @addtogroup math

//! @{
/**
 * @brief Find solutions for roots of real and complex polynomial equations.
 *
 * The solutions are found analytically if the polynomial is of maximum order 3.
 * For polynomials of order 4 and higher, Laguerre's Method is used to find
 * roots until the polynomial can be deflated to order 3.
 * The remaining roots will then be found analytically.
 *
 * Some Polynomials are particularly easy to solve. A polynomial of the form
 * \f$ a x^n + b = 0\f$
 * will be solved by taking the n'th roots of \f$-\frac{b}{a}\f$ directly, giving n distinct roots in the complex plane.
 *
 * To illustrate the procedure, consider the equation:
 * \f$ 10^{-15} x^8 - 10^{-15} x^7 + x^7 + 2 x^6 - x^4 - 2x^3 + 10^{-15} x= 0\f$
 *
 * The solver will use the following procedure (here with the precision \f$ \epsilon = 10^{-14}\f$):
 *
 * 1. Remove terms that are small compared to \f$\epsilon\f$: \f$ x^7 + 2 x^6 - x^4 - 2x^3 = 0\f$
 *
 * 2. Find zero roots and reduce the order: There is a triple root in x = 0 and the remaining polynomial becomes: \f$ x^4 + 2 x^3 - x - 2 = 0\f$.
 *
 * 3. Use Laguerre to find a root of \f$ x^4 + 2 x^3 - x - 2 = 0\f$
 *
 * Depending on the initial guess for Laguerre, different roots might be found first.
 * The algorithm will proceed differently depending on the found root:
 *
 * 1. If root x=-2 is found, remaining polynomial after deflation is \f$ x^3 -1 = 0\f$.
 * The roots are found directly as the cubic root of 1, which is three distinct roots in the complex plane (one is on the real axis).
 *
 * 2. If root x=1 is found, remaining polynomial after deflation is \f$ x^3 + 3 x^2 +3 x + 2 = 0\f$.
 * The roots are found analytically, giving one real root x=-2 and two complex conjugate roots \f$x = -0.5 \pm \frac{\sqrt{3}}{2} i\f$.
 *
 * 3. If other roots than x=1 or x=-2 is found (a complex root), remaining polynomial is a third order polynomial with complex coefficients. This polynomial is solved analytically to give remaining two real roots, and one remaining complex root.
 *
 * Notice that cases 2+3 requires analytical solution of the third order polynomial equation.
 * For higher order polynomials Laguerre would need to be used to find the next root.
 * In this case it is particularly lucky to hit case 1, as this gives the solutions right away no matter what order the remaining polynomial is.
 */
class PolynomialSolver {
public:
	/**
	 * @brief Create a solver for a polynomial with real coefficients.
	 * @param polynomial [in] the polynomial to find roots for.
	 */
	PolynomialSolver(const Polynomial<double>& polynomial);

	/**
	 * @brief Create a solver for a polynomial with complex coefficients.
	 * @param polynomial [in] the polynomial to find roots for.
	 */
	PolynomialSolver(const Polynomial<std::complex<double> >& polynomial);

	/**
	 * @brief Destructor
	 */
	virtual ~PolynomialSolver();

	/**
	 * @brief Use a specific initial guess for a root.
	 * @param guess [in] a complex initial guess for the algorithm.
	 */
	void setInitialGuess(std::complex<double> guess = 0);

	/**
	 * @brief Get all real solutions of the equation.
	 * @param epsilon [in] the root is considered a real root if \f$ |im(x)| \leq 2 \epsilon |real(x)|\f$ .
	 * @return a list of real solutions.
	 * @throws rw::common::Exception if the Laguerre method fails, or the maximum number of iterations has been reached.
	 * @see PolynomialSolver for more details about the method used.
	 */
	std::vector<double> getRealSolutions(double epsilon = 1.0e-14);

	/**
	 * @brief Get all solutions of the equation including complex solutions.
	 * @param epsilon [in] highest order coefficients will be removed if they have absolute real and imaginary values less than \f$ \epsilon \f$ .
	 * @return a list of complex solutions.
	 * @throws rw::common::Exception if the Laguerre method fails, or the maximum number of iterations has been reached.
	 * @see PolynomialSolver for more details about the method used.
	 */
	virtual std::vector<std::complex<double> > getSolutions(double epsilon = 1.0e-14);

	/**
	 * @brief Set the number of iterations to take in the Laguerre method.
	 * @param iterations [in] the maximum number of iterations (default is 10).
	 */
	void setLaguerreIterations(unsigned int iterations = 10);

private:
	bool analyticalSolutions(std::vector<std::complex<double> >& res, double EPS) const;
	std::complex<double> laguerre(std::complex<double> x) const;

	Polynomial<std::complex<double> > _polynomial;
	PolynomialSolver* _derivative;
	std::complex<double> _guess;
	bool _isComplex;
	unsigned int _iterations;
};
//! @}
} /* namespace math */
} /* namespace rw */
#endif /* RW_MATH_POLYNOMIALSOLVER_HPP_ */
