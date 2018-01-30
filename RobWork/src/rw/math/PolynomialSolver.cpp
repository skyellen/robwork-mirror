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

#include <rw/common/macros.hpp>
#include <rw/math/Constants.hpp>

#include <limits>
#include <complex>
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
		if (std::fabs(x.imag()) <= 2.*EPS*std::fabs(x.real())) {
			res.push_back(x.real());
		}
	}

	return res;
}

std::vector<std::complex<double> > PolynomialSolver::getSolutions(const double EPS) {
	std::vector<std::complex<double> > res;

	// Remove highest order coefficients if close to zero
	PolynomialSolver* polZ = this;
	PolynomialSolver* pol = this;
	if (pol->_polynomial.order() <= 0)
		return res;
	if(std::abs(pol->_polynomial[pol->_polynomial.order()]) <= EPS) {
		std::size_t zeroIdx = 0;
		for (std::size_t i = 1; i < pol->_polynomial.order(); i++) {
			if (std::abs(pol->_polynomial[pol->_polynomial.order()-i]) <= EPS) {
				zeroIdx = i;
			} else {
				break;
			}
		}
		Polynomial<std::complex<double> > newPol(pol->_polynomial.order()-1-zeroIdx);
		for (std::size_t i = 0; i < pol->_polynomial.order()-zeroIdx; i++) {
			newPol[i] = pol->_polynomial[i];
		}
		if (newPol.order() <= 0)
			return res;
		pol = new PolynomialSolver(newPol);
		polZ = new PolynomialSolver(newPol);
	}

	// Reduce order if lowest order coefficients is close to zero
	if(std::abs(pol->_polynomial[0]) <= EPS) {
		std::size_t zeroIdx = 0;
		for (std::size_t i = 1; i <= pol->_polynomial.order(); i++) {
			if (std::abs(pol->_polynomial[i]) <= EPS) {
				zeroIdx = i;
			} else {
				break;
			}
		}
		for (std::size_t i = 0; i <= zeroIdx; i++)
			res.push_back(0.);
		Polynomial<std::complex<double> > newPol(pol->_polynomial.order()-1-zeroIdx);
		for (std::size_t i = 0; i < pol->_polynomial.order()-zeroIdx; i++) {
			newPol[i] = pol->_polynomial[i+zeroIdx+1];
		}
		if (pol != this)
			delete pol;
		if (polZ != this)
			delete polZ;
		if (newPol.order() <= 0)
			return res;
		pol = new PolynomialSolver(newPol);
		polZ = new PolynomialSolver(newPol);
	}

	// Try to solve analytically if the polynomial is simple enough
	bool analyticSolutions = pol->analyticalSolutions(res, EPS);

	if (!analyticSolutions) {
		// Find tentative roots
		for (std::size_t i = 0; i < polZ->_polynomial.order() && !analyticSolutions; i++) {
			// No analytical solution can be found yet, so use Laguerre to find a root
			std::complex<double> x = _guess;
			x = pol->laguerre(x);
			res.push_back(x);
			PolynomialSolver* old = pol;
			pol = new PolynomialSolver(pol->_polynomial.deflate(x));
			if (old != this)
				delete old;
			analyticSolutions = pol->analyticalSolutions(res, EPS);
		}

		// Polish
		for (std::size_t i = 0; i < polZ->_polynomial.order(); i++) {
			res[i] = polZ->laguerre(res[i]);
		}
	}

	if (pol != this)
		delete pol;
	if (polZ != this)
		delete polZ;

	return res;
}

void PolynomialSolver::setLaguerreIterations(unsigned int iterations) {
	_iterations = iterations;
}

bool PolynomialSolver::analyticalSolutions(std::vector<std::complex<double> >& res, const double EPS) const {
	// Check if polynomial is of the trivial format a x^n + b = 0
	bool trivial = true;
	for (std::size_t i = 1; i < _polynomial.order() && trivial; i++) {
		if (std::abs(_polynomial[i]) > EPS)
			trivial = false;
	}
	if (trivial) {
		const Polynomial<std::complex<double> >& p = _polynomial;
		const double order = static_cast<double>(p.order());
		const std::complex<double> val = -p[0]/p[order];
		const double z = std::pow(std::abs(val),1./order);
		const double arg = std::arg(val);
		if (p.order() == 1) {
			res.push_back(-p[0]/p[1]);
		} else if (p.order()%2 == 0) {
			for (std::size_t i = 0; i < p.order()/2; i ++) {
				const double real = z*std::cos((arg+Pi*2*i)/order);
				const double imag = z*std::sin((arg+Pi*2*i)/order);
				res.push_back(std::complex<double>( real, imag));
				res.push_back(std::complex<double>(-real,-imag));
			}
		} else {
			for (std::size_t i = 0; i < p.order(); i ++) {
				const double real = z*std::cos((arg+Pi*2*i)/order);
				const double imag = z*std::sin((arg+Pi*2*i)/order);
				res.push_back(std::complex<double>(real, imag));
			}
		}
		return true;
	} else if (_polynomial.order() <= 3) { // Check if polynomial is degree 3 or less and solve analytically
		const Polynomial<std::complex<double> >& p = _polynomial;
		// For order 0 there will be no solution
		// Order 1 is a trivial case
		if (_polynomial.order() == 2) {
			const std::complex<double> K = std::sqrt(p[1]*p[1]-p[2]*p[0]*4.);
			res.push_back((-p[1]+K)/(p[2]*2.));
			res.push_back((-p[1]-K)/(p[2]*2.));
		} else if (_polynomial.order() == 3) {
			const std::complex<double> a3 = p[3]*3.;
			const std::complex<double> a3a3 = a3*a3;
			const std::complex<double> a4 = p[3]*4.;
			const std::complex<double> bc = p[2]*p[1];
			const std::complex<double> bPow3 = p[2]*p[2]*p[2];
			const std::complex<double> cPow3 = p[1]*p[1]*p[1];
			const std::complex<double> D = p[3]*bc*p[0]*18.-bPow3*p[0]*4.+bc*bc-a4*cPow3-a3a3*p[0]*p[0]*3.; // discriminant
			const std::complex<double> D0 = p[2]*p[2]-p[1]*a3;
			if (std::abs(D) < EPS) {
				if (std::abs(D0) < EPS) {
					const std::complex<double> root = -p[2]/a3;
					res.push_back(root);
					res.push_back(root);
					res.push_back(root);
				} else {
					const std::complex<double> root = (a3*p[0]*3.-bc)/(D0*2.);
					res.push_back(root);
					res.push_back(root);
					res.push_back((a4*bc-a3a3*p[0]-bPow3)/(D0*p[3]));
				}
			} else {
				const std::complex<double> D1 = bPow3*2.-a3*bc*3.+a3a3*p[0]*3.;
				const std::complex<double> K = D1*D1-D0*D0*D0*4.;
				const std::complex<double> Cin = (D1+std::sqrt(K))/2.;
				const double z = std::pow(std::abs(Cin),1./3.);
				const double arg = std::arg(Cin);
				const std::complex<double> C0(z*std::cos(arg/3),z*std::sin(arg/3));
				const std::complex<double> C1 = C0*std::complex<double>(-1./2.,std::sqrt(3.)/2.);
				const std::complex<double> C2 = C0*std::complex<double>(-1./2.,-std::sqrt(3.)/2.);
				res.push_back(-p[2]/a3-(C0+D0/C0)/a3);
				res.push_back(-p[2]/a3-(C1+D0/C1)/a3);
				res.push_back(-p[2]/a3-(C2+D0/C2)/a3);
			}
		}
		return true;
	}
	return false;
}

std::complex<double> PolynomialSolver::laguerre(std::complex<double> x) const {
	const double n = 1.0+_polynomial.order();
	for (std::size_t i = 1; i <= _iterations; i++) {
		double err;
		std::complex<double> val = _polynomial.evaluate(x,err);
		if (std::abs(val) <= err)
			return x;
		const std::vector<std::complex<double> > der = _polynomial.evaluateDerivatives(x,2);
		const std::complex<double> G = der[1]/val;
		const std::complex<double> Gsq = G*G;
		const std::complex<double> H = Gsq-der[2]/val;
		const std::complex<double> sq = std::sqrt((n-2)*((n-1)*H-Gsq));
		const std::complex<double> denPlus = G+sq;
		const std::complex<double> denMinus = G-sq;
		const std::complex<double> den = (std::abs(denPlus) > std::abs(denMinus)) ? denPlus : denMinus;
		if (std::abs(den) == 0)
			RW_THROW("PolynomialSolver::laguerre failed du to zero denominator!");
		const std::complex<double> a = (n-1)/den;
		std::complex<double> xNew = x - a;
		if (xNew == x)
			return xNew;
		x = xNew;
	}

	//RW_THROW("PolynomialSolver::laguerre failed to find solution - too many iterations.");
	RW_WARN("PolynomialSolver::laguerre failed to find solution - too many iterations.");
	return 0;
}
