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

#include "PolynomialSolver.hpp"
#include "Polynomial.hpp"

#include <limits>
#include <complex>

#include <rw/common/macros.hpp>
#include <stddef.h>

using namespace rw::math;

PolynomialSolver::PolynomialSolver(const Polynomial<>& polynomial):
	_polynomial(polynomial.order()),
	_derivative(NULL),
	_guess(0),
	_isComplex(false),
	_iterations(10)
{
	for (std::size_t i = 0; i <= polynomial.order(); i++)
		_polynomial[i] = polynomial[i];
}

PolynomialSolver::PolynomialSolver(const Polynomial<std::complex<double> >& polynomial):
	_polynomial(polynomial),
	_derivative(NULL),
	_guess(0),
	_isComplex(true),
	_iterations(10)
{
}

PolynomialSolver::~PolynomialSolver() {
	if (_derivative != NULL)
		delete _derivative;
}

void PolynomialSolver::setInitialGuess(std::complex<double> guess) {
	_guess = guess;
}

std::vector<double> PolynomialSolver::getRealSolutions(const double EPS) {
	std::vector<std::complex<double> > sol = getSolutions(EPS);
	std::vector<double> res;

	for (std::size_t i = 0; i < sol.size(); i++) {
		std::complex<double> x = sol[i];
		if (fabs(x.imag()) <= 2.*EPS*fabs(x.real())) {
			res.push_back(x.real());
		}
	}

	return res;
}

std::vector<std::complex<double> > PolynomialSolver::getSolutions(const double EPS) {
	std::vector<std::complex<double> > res;

	// Remove highest order coefficients if close to zero
	PolynomialSolver* polZ;
	PolynomialSolver* pol;
	if(std::fabs(_polynomial[_polynomial.order()].real()) <= EPS && std::fabs(_polynomial[_polynomial.order()].imag()) <= EPS) {
		std::size_t zeroIdx = 0;
		for (std::size_t i = 1; i < _polynomial.order(); i++) {
			if (std::fabs(_polynomial[_polynomial.order()-i].real()) <= EPS && std::fabs(_polynomial[_polynomial.order()-i].imag()) <= EPS) {
				zeroIdx = i;
			} else {
				break;
			}
		}
		Polynomial<std::complex<double> > newPol(_polynomial.order()-1-zeroIdx);
		for (std::size_t i = 0; i < _polynomial.order()-zeroIdx; i++) {
			newPol[i] = _polynomial[i];
		}
		pol = new PolynomialSolver(newPol);
		polZ = new PolynomialSolver(newPol);
	} else {
		polZ = this;
		pol = this;
	}

	// Find tentative roots
	for (std::size_t i = 0; i < polZ->_polynomial.order(); i++) {
		std::complex<double> x = _guess;
		x = pol->laguerre(x);
		res.push_back(x);
		PolynomialSolver* old = pol;
		pol = new PolynomialSolver(pol->_polynomial.deflate(x));
		if (old != this)
			delete old;
	}
	if (pol != this)
		delete pol;

	// Polish
	for (std::size_t i = 0; i < polZ->_polynomial.order(); i++) {
		res[i] = polZ->laguerre(res[i]);
	}
	if (polZ != this)
		delete polZ;

	return res;
}

void PolynomialSolver::setLaguerreIterations(unsigned int iterations) {
	_iterations = iterations;
}

std::complex<double> PolynomialSolver::laguerre(std::complex<double> x) const {
	const double n = 1.0+_polynomial.order();
	for (std::size_t i = 1; i <= _iterations; i++) {
		double err;
		std::complex<double> val = _polynomial.evaluate(x,err);
		if (abs(val) <= err)
			return x;
		const std::vector<std::complex<double> > der = _polynomial.evaluateDerivatives(x,2);
		const std::complex<double> G = der[1]/val;
		const std::complex<double> Gsq = G*G;
		const std::complex<double> H = Gsq-der[2]/val;
		const std::complex<double> sq = std::sqrt((n-2)*((n-1)*H-Gsq));
		const std::complex<double> denPlus = G+sq;
		const std::complex<double> denMinus = G-sq;
		const std::complex<double> den = (abs(denPlus) > abs(denMinus)) ? denPlus : denMinus;
		if (abs(den) == 0)
			RW_THROW("PolynomialSolver::laguerre failed du to zero denominator!");
		const std::complex<double> a = (n-1)/den;
		std::complex<double> xNew = x - a;
		if (xNew == x)
			return xNew;
		x = xNew;
	}

	RW_THROW("PolynomialSolver::laguerre failed to find solution - too many iterations.");
	return 0;
}
