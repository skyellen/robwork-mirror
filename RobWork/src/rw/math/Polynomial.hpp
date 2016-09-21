/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_MATH_POLYNOMIAL_HPP_
#define RW_MATH_POLYNOMIAL_HPP_

#include <rw/common/macros.hpp>
#include <rw/common/Serializable.hpp>

#include <vector>
#include <limits>
#include <cmath>

/**
 * @file Polynomial.hpp
 *
 * \copydoc rw::math::Polynomial
 */

namespace rw {
namespace math {
//! @addtogroup math

//! @{
/**
 * @brief Representation of a real polynomial.
 *
 * Representation of a polynomial of the following form:
 *
 * @f$
 *  f(x) = c_n x^n + c_(n-1) x^(n-1) + c_2 x^2 + c_1 x + c_0
 * @f$
 *
 * The polynomial is represented as a list of coefficients ordered from lowest-order term to highest-order term, \f${c_0,c_1,...,c_n}\f$.
 */
template<typename T = double>
class Polynomial {
public:
	/**
	 * @brief Create polynomial with coefficients initialized to zero.
	 * @param order [in] the order of the polynomial.
	 */
	Polynomial(std::size_t order):
		_EPS(std::numeric_limits<T>::epsilon()),
		_coef(std::vector<T>(order+1,0))
	{
	}

	/**
	 * @brief Create polynomial from vector.
	 * @param coefficients [in] the coefficients ordered from lowest-order term to highest-order term.
	 */
	Polynomial(const std::vector<T> &coefficients):
		_EPS(std::numeric_limits<T>::epsilon()),
		_coef(coefficients)
	{
	}

	/**
	 * @brief Create polynomial from other polynomial.
	 * @param p [in] the polynomial to copy.
	 */
	Polynomial(const Polynomial<T> &p):
		_EPS(std::numeric_limits<T>::epsilon()),
		_coef(std::vector<T>(p.order()+1))
	{
		for (std::size_t i = 0; i <= p.order(); i++)
			_coef[i] = p[i];
	}

	/**
	 * @brief Destructor
	 */
	virtual ~Polynomial() {
	}

	/**
	 * @brief Get the order of the polynomial (the highest power).
	 * @return the order.
	 */
	std::size_t order() const {
		return _coef.size()-1;
	}

	/**
	 * @brief Increase the order of this polynomial.
	 * @param increase [in] how much to increase the order (default is 1).
	 */
	void increaseOrder(std::size_t increase = 1) {
		_coef.resize(_coef.size()+increase,0);
	}

	/**
	 * @brief Evaluate the polynomial using Horner's Method.
	 * @param x [in] the input parameter.
	 * @return the value \f$f(x)\f$.
	 */
	T evaluate(const T &x) const {
		// Horner's Method
		T res = _coef.back();
		for (int i = static_cast<int>(_coef.size()-2); i >= 0; i--)
			res = _coef[i]+res*x;
		return res;
	}

	/**
	 * @brief Evaluate the polynomial using Horner's Method.
	 * @param x [in] the input parameter.
	 * @param err [out] estimate of the absolute error.
	 * @return the value \f$f(x)\f$.
	 * @note Error is the absolute size of the interval where \f$f(x)\f$ can be, assuming coefficients are exact.
	 */
	T evaluate(const T &x, T& err) const {
		// Horner's Method
		T res = _coef.back();
		double errCoef = 0; // Error due to finite precision coefficients
		double errX = 0; // Error due to finite precision x value
		double errComb = 0; // Combinational error of error both in coefficients and x (magnitude very small - eps*eps)
		errCoef = fabs(res);
		const double dX = fabs(x)*_EPS;
 		for (int i = static_cast<int>(_coef.size()-2); i >= 0; i--) {
 			errX = fabs(res)*dX+errX*fabs(x)+errX*dX;
			res = _coef[i]+res*x;
			errComb = errComb*fabs(x)+errComb*dX+errCoef*_EPS*dX;
			errCoef = fabs(_coef[i]) + fabs(x)*errCoef;
		}
 		errCoef *= _EPS;
 		err = errCoef+errX+errComb;
		return res;
	}

	/**
	 * @brief Evaluate a the derivatives of the polynomial using Horner's Method.
	 * @param x [in] the input parameter.
	 * @param n [in] the number of derivatives to find (default is the first derivative only)
	 * @return a vector of values \f${f(x),\dot{f}(x),\ddot{f}(x),\cdots}\f$.
	 */
	std::vector<T> evaluateDerivatives(const T &x, std::size_t n = 1) const {
		// Horner's Method
		std::vector<T> res(n+1,0);
		res[0] = _coef.back();
		for (int i = static_cast<int>(_coef.size()-2); i >= 0; i--) {
			int minJ = static_cast<int>(std::min<std::size_t>(n,_coef.size()-1-i));
			for (int j = minJ; j > 0; j--) {
				res[j] = res[j-1]+res[j]*x;
			}
			res[0] = _coef[i]+res[0]*x;
		}
		T k = 1;
		for (std::size_t i = 2; i <= n; i++) {
			k *= i;
			res[i] *= k;
		}
		return res;
	}

	/**
	 * @brief Evaluate a the first derivative of the polynomial using Horner's Method.
	 * @param x [in] the input parameter.
	 * @param err [out] estimate of the absolute errors.
	 * @param n [in] the number of derivatives to find (default is the first derivative only)
	 * @return the value \f$\dot{f}(x)\f$.
	 * @note Error is the absolute size of the interval where \f$f(x)\f$ can be, assuming coefficients are exact.
	 */
	std::vector<T> evaluateDerivatives(const T &x, std::vector<T>& err, std::size_t n = 1) const {
		// Horner's Method
		err.resize(n+1);
		for (std::size_t i = 0; i < err.size(); i++)
			err[i] = 0;
		std::vector<T> res(n+1,0);
		res[0] = _coef.back();
		err[0] = fabs(res[0]);
		for (int i = static_cast<int>(_coef.size()-2); i >= 0; i--) {
			int minJ = static_cast<int>(std::min<std::size_t>(n,_coef.size()-1-i));
			for (int j = minJ; j > 0; j--) {
				res[j] = res[j-1]+res[j]*x;
				err[j] = fabs(res[j-1])+fabs(x)*err[j];
			}
			res[0] = _coef[i]+res[0]*x;
			err[0] = fabs(_coef[i])+fabs(x)*err[0];
		}
		std::size_t k = 1;
		for (std::size_t i = 2; i <= n; i++) {
			k *= i;
			res[i] *= k;
			err[i] *= k;
		}
		for (std::size_t i = 0; i <= n; i++) {
			err[i] *= _EPS;
		}
		return res;
	}

	/**
	 * @brief Perform deflation of polynomial.
	 * @param x [in] a root of the polynomial.
	 * @return a new polynomial of same order minus one.
	 * @note There is no check that the given root is in fact a root of the polynomial.
	 */
	Polynomial<T> deflate(const T &x) const {
		// Horner Method
		std::size_t no = order()-1;
		Polynomial<T> res(no);
		res[no] = _coef.back();
		for (int i = (int)no-1; i >= 0; i--) {
			res[i] = x*res[i+1]+_coef[i+1];
		}
		return res;
	}

	/**
	 * @brief Get the derivative polynomial.
	 * @param n [in] gives the n'th derivative (default is n=1).
	 * @return a new polynomial of same order minus one.
	 * @note To evaluate derivatives use the evaluate derivative method which is more precise.
	 */
	Polynomial<T> derivative(std::size_t n = 1) const {
		if (n == 0)
			return *this;
		std::size_t no = order()-1;
		Polynomial<T> der(no);
		for (std::size_t i = 1; i <= order(); i++)
			der[i-1] = _coef[i]*i;
		return der.derivative(n-1);
	}

	/**
	 * @name Coefficient access operators.
	 * Operators used to access coefficients.
	 */
	///@{

	/**
	 * @brief Get specific coefficient.
	 * @param i [in] the power of the term to get coefficient for.
	 * @return the coefficient.
	 */
	const T& operator()(size_t i) const {
		if (i > order()) {
			std::stringstream str;
			str << "Polynomial of order " << order() << " has no coefficient with index " << i;
			RW_THROW(str.str());
		}
		return _coef[i];
	}

	/**
	 * @brief Get specific coefficient.
	 * @param i [in] the power of the term to get coefficient for.
	 * @return the coefficient.
	 */
	T& operator()(size_t i) {
		if (i > order()) {
			std::stringstream str;
			str << "Polynomial of order " << order() << " has no coefficient with index " << i;
			RW_THROW(str.str());
		}
		return _coef[i];
	}

	/**
	 * @brief Get specific coefficient.
	 * @param i [in] the power of the term to get coefficient for.
	 * @return the coefficient.
	 */
	const T& operator[](size_t i) const {
		if (i > order()) {
			std::stringstream str;
			str << "Polynomial of order " << order() << " has no coefficient with index " << i;
			RW_THROW(str.str());
		}
		return _coef[i];
	}

	/**
	 * @brief Get specific coefficient.
	 * @param i [in] the power of the term to get coefficient for.
	 * @return the coefficient.
	 */
	T& operator[](size_t i) {
		if (i > order()) {
			std::stringstream str;
			str << "Polynomial of order " << order() << " has no coefficient with index " << i;
			RW_THROW(str.str());
		}
		return _coef[i];
	}
	///@}

	/**
	 * @name Arithmetic operators between polynomial and scalars.
	 * Operators used to do arithmetic with scalars.
	 */
	///@{

	/**
	 * @brief Scalar addition
	 * @param s [in] scalar to add.
	 * @return new polynomial after addition.
	 */
	const Polynomial<T> operator+(T s) const
	{
		Polynomial<T> pol = *this;
		pol[0] += s;
		return pol;
	}

	/**
	 * @brief Scalar subtraction
	 * @param s [in] scalar to subtract.
	 * @return new polynomial after subtraction.
	 */
	const Polynomial<T> operator-(T s) const
	{
		Polynomial<T> pol = *this;
		pol[0] -= s;
		return pol;
	}

	/**
	 * @brief Scalar multiplication
	 * @param s [in] scalar to multiply with.
	 * @return new polynomial after multiplication.
	 */
	const Polynomial<T> operator*( T s) const
	{
		Polynomial<T> pol(order());
		for (std::size_t i = 0; i <= order(); i++) {
			pol[i] = _coef[i]*s;
		}
		return pol;
	}

	/**
	 * @brief Scalar division
	 * @param s [in] scalar to divide with.
	 * @return new polynomial after division.
	 */
	const Polynomial<T> operator/(T s) const
	{
		Polynomial<T> pol(order());
		for (std::size_t i = 0; i <= order(); i++) {
			pol[i] = _coef[i]/s;
		}
		return pol;
	}

	/**
	 * @brief Scalar addition
	 * @param s [in] scalar to add.
	 * @return same polynomial with coefficients changed.
	 */
	Polynomial<T>& operator+=(T s)
	{
		_coef[0] += s;
		return *this;
	}

	/**
	 * @brief Scalar subtraction
	 * @param s [in] scalar to subtract.
	 * @return same polynomial with coefficients changed.
	 */
	Polynomial<T>& operator-=(T s)
	{
		_coef[0] -= s;
		return *this;
	}

	/**
	 * @brief Scalar multiplication
	 * @param s [in] the scalar to multiply with.
	 * @return reference to same polynomial with changed coefficients.
	 */
	Polynomial<T>& operator*=(T s) {
		for (std::size_t i = 0; i <= order(); i++) {
			_coef[i] *= s;
		}
		return *this;
	}

	/**
	 * @brief Scalar division
	 * @param s [in] the scalar to divide with.
	 * @return reference to same polynomial with changed coefficients.
	 */
	Polynomial<T>& operator/=(T s) {
		for (std::size_t i = 0; i <= order(); i++) {
			_coef[i] /= s;
		}
		return *this;
	}

	/**
	 * @brief Scalar multiplication
	 * @param s [in] scalar to multiply with.
	 * @param v [in] polynomial to multiply with.
	 * @return new polynomial after multiplication.
	 */
	friend const Polynomial<T> operator*(T s, const Polynomial<T>& p)
	{
		Polynomial<T> pol(p.order());
		for (std::size_t i = 0; i <= p.order(); i++) {
			pol[i] = p[i]*s;
		}
		return pol;
	}
	///@}

	/**
	 * @name Arithmetic operators between polynomials.
	 * Operators used to do arithmetic between two polynomials.
	 */
	///@{

	/**
	 * @brief Polynomial subtraction.
	 * @param b [in] polynomial of to subtract.
	 * @return new polynomial after subtraction.
	 */
	const Polynomial<T> operator-(const Polynomial<T>& b) const
	{
		std::size_t ord = std::max<std::size_t>(order(),b.order());
		Polynomial<T> pol(ord);
		for (std::size_t i = 0; i <= order(); i++) {
			pol[i] = _coef[i];
		}
		for (std::size_t i = 0; i <= b.order(); i++) {
			pol[i] -= b[i];
		}
		return pol;
	}

	/**
	 * @brief Polynomial subtraction.
	 * @param b [in] polynomial to subtract.
	 * @return same polynomial with different coefficients after subtraction.
	 */
	Polynomial<T>& operator-=(const Polynomial<T>& b) {
		std::size_t ord = std::max<std::size_t>(order(),b.order());
		if (ord > order())
			increaseOrder(ord-order());
		for (std::size_t i = 0; i <= b.order(); i++) {
			_coef[i] -= b[i];
		}
		return *this;
	}

	/**
	 * @brief Polynomial addition.
	 * @param b [in] polynomial to add.
	 * @return new polynomial after addition.
	 */
	const Polynomial<T> operator+(const Polynomial<T>& b) const
	{
		std::size_t ord = std::max<std::size_t>(order(),b.order());
		Polynomial<T> pol(ord);
		for (std::size_t i = 0; i <= order(); i++) {
			pol[i] = _coef[i];
		}
		for (std::size_t i = 0; i <= b.order(); i++) {
			pol[i] += b[i];
		}
		return pol;
	}

	/**
	 * @brief Polynomial addition.
	 * @param b [in] polynomial to add.
	 * @return same polynomial with different coefficients after addition.
	 */
	Polynomial<T>& operator+=(const Polynomial<T>& b) {
		std::size_t ord = std::max<std::size_t>(order(),b.order());
		if (ord > order())
			increaseOrder(ord-order());
		for (std::size_t i = 0; i <= b.order(); i++) {
			_coef[i] += b[i];
		}
		return *this;
	}

	/**
	 * @brief Assignment.
	 * @param b [in] the polynomial to take coefficients from.
	 * @return true if equal, false if not.
	 */
	void operator=(const Polynomial<T>& b) {
		_coef.resize(b.order()+1);
		for(size_t i=0;i<=b.order();i++)
			_coef[i] = b[i];
	}

	///@}

	/**
	 * @brief Negate coefficients.
	 * @return new polynomial with coefficients negated.
	 */
	const Polynomial<T> operator-() const
	{
		Polynomial<T> pol(order());
		for (std::size_t i = 0; i <= order(); i++) {
			pol[i] = -_coef[i];
		}
		return pol;
	}

	/**
	 * @brief Printing polynomial to stream.
	 * @param out [in/out] the stream to write to.
	 * @param p [in] the polynomail to print.
	 * @return the same ostream as out parameter.
	 */
	friend std::ostream& operator<<(std::ostream& out, const Polynomial<T>& p)
	{
		out << "Polynomial: ";
		for(size_t i=p.order();i>0;i--)
			out << p[i] << " x^" << i << " + ";
		out << p[0];
		return out;
	}

	/**
	 * @brief Check if polynomials are equal.
	 * @param b [in] the polynomial to compare with.
	 * @return true if equal, false if not.
	 */
	bool operator==(const Polynomial<T>& b) const {
		for(size_t i=0;i<=order();i++)
			if(_coef[i]!=b[i])
				return false;
		return true;
	}

	/**
	 * @brief Cast to other type.
	 * @return a new polynomial after cast to new type.
	 */
	template<class Q>
	const Polynomial<Q> cast()
	{
		Polynomial<Q> pol(order());
		for (std::size_t i = 0; i <= order(); i++) {
			pol[i] = static_cast<Q>(_coef[i]);
		}
		return pol;
	}

private:
	const double _EPS;
	std::vector<T> _coef;
};
//! @}
} /* namespace math */
} /* namespace rw */

namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
	template<> void write(const rw::math::Polynomial<double>& tmp, rw::common::OutputArchive& oar, const std::string& id);
	template<> void write(const rw::math::Polynomial<float>& tmp, rw::common::OutputArchive& oar, const std::string& id);
	template<> void read(rw::math::Polynomial<double>& tmp, rw::common::InputArchive& iar, const std::string& id);
	template<> void read(rw::math::Polynomial<float>& tmp, rw::common::InputArchive& iar, const std::string& id);
}}} // end namespaces

#endif /* RW_MATH_POLYNOMIAL_HPP_ */
