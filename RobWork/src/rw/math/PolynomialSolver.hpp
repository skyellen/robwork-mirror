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
//! @addtogroup rw_math

//! @{
/**
 * @brief Solution for roots of real and complex polynomial equations with Laguerre's Method.
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
	 */
	std::vector<double> getRealSolutions(double epsilon = 1.0e-14);

	/**
	 * @brief Get all solutions of the equation including complex solutions.
	 * @param epsilon [in] highest order coefficients will be removed if they have absolute real and imaginary values less than \f$ \epsilon \f$ .
	 * @return a list of complex solutions.
	 * @throws rw::common::Exception if the Laguerre method fails, or the maximum number of iterations has been reached.
	 */
	virtual std::vector<std::complex<double> > getSolutions(double epsilon = 1.0e-14);

	/**
	 * @brief Set the number of iterations to take in the Laguerre method.
	 * @param iterations [in] the maximum number of iterations (default is 10).
	 */
	void setLaguerreIterations(unsigned int iterations = 10);

private:
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
