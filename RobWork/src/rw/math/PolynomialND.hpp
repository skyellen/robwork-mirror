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

#ifndef RW_MATH_POLYNOMIALND_HPP_
#define RW_MATH_POLYNOMIALND_HPP_

/**
 * @file PolynomialND.hpp
 *
 * \copydoc rw::math::PolynomialND
 */

#include <rw/common/macros.hpp>

#include <Eigen/Core>

#include <sstream>
#include <vector>

namespace rw {
namespace math {
//! @addtogroup math

//! @{
/**
 * @brief Representation of a polynomial that can have non-scalar coefficients (polynomial matrix).
 *
 * Representation of a polynomial of the following form:
 *
 * @f$
 *  f(x) = C_n x^n + C_(n-1) x^(n-1) + C_2 x^2 + C_1 x + C_0
 * @f$
 *
 * The polynomial is represented as a list of coefficients ordered from lowest-order term to highest-order term, \f${c_0,c_1,...,c_n}\f$.
 */
template<typename Coef, typename Scalar = double>
class PolynomialND {
public:
	/**
	 * @brief Create polynomial with uninitialized coefficients.
	 * @param order [in] the order of the polynomial.
	 */
	PolynomialND(std::size_t order):
		_coef(std::vector<Coef>(order+1))
	{
	}

	/**
	 * @brief Create polynomial from vector.
	 * @param coefficients [in] the coefficients ordered from lowest-order term to highest-order term.
	 */
	PolynomialND(const std::vector<Coef> &coefficients):
		_coef(coefficients)
	{
	}

	/**
	 * @brief Create polynomial from other polynomial.
	 * @param p [in] the polynomial to copy.
	 */
	PolynomialND(const PolynomialND<Coef, Scalar> &p):
		_coef(std::vector<Coef>(p.order()+1))
	{
		for (std::size_t i = 0; i <= p.order(); i++)
			_coef[i] = p[i];
	}

	/**
	 * @brief Destructor
	 */
	virtual ~PolynomialND() {
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
	Coef evaluate(const Scalar &x) const {
		// Horner's Method
		Coef res = _coef.back();
		for (int i = static_cast<int>(_coef.size()-2); i >= 0; i--)
			res = _coef[i]+res*x;
		return res;
	}

	/**
	 * @brief Evaluate the first \b n derivatives of the polynomial using Horner's Method.
	 * @param x [in] the input parameter.
	 * @param n [in] the number of derivatives to find (default is the first derivative only)
	 * @return a vector of values \f${f(x),\dot{f}(x),\ddot{f}(x),\cdots}\f$.
	 */
	std::vector<Coef> evaluateDerivatives(const Scalar &x, std::size_t n = 1) const {
		// Horner's Method
		std::vector<Coef> res(n+1,0);
		res[0] = _coef.back();
		for (int i = static_cast<int>(_coef.size()-2); i >= 0; i--) {
			int minJ = static_cast<int>(std::min<std::size_t>(n,_coef.size()-1-i));
			for (int j = minJ; j > 0; j--) {
				res[j] = res[j-1]+res[j]*x;
			}
			res[0] = _coef[i]+res[0]*x;
		}
		Scalar k = 1;
		for (std::size_t i = 2; i <= n; i++) {
			k *= i;
			res[i] *= k;
		}
		return res;
	}

	/**
	 * @brief Perform deflation of polynomial.
	 * @param x [in] a root of the polynomial.
	 * @return a new polynomial of same order minus one.
	 * @note There is no check that the given root is in fact a root of the polynomial.
	 */
	PolynomialND<Coef, Scalar> deflate(const Scalar &x) const {
		// Horner Method
		std::size_t no = order()-1;
		PolynomialND<Coef, Scalar> res(no);
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
	PolynomialND<Coef, Scalar> derivative(std::size_t n = 1) const {
		if (n == 0)
			return *this;
		std::size_t no = order()-1;
		PolynomialND<Coef, Scalar> der(no);
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
	const Coef& operator()(std::size_t i) const {
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
	Coef& operator()(size_t i) {
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
	const Coef& operator[](size_t i) const {
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
	Coef& operator[](size_t i) {
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
	const PolynomialND<Coef, Scalar> operator+(Scalar s) const
	{
		PolynomialND<Coef, Scalar> pol = *this;
		pol[0] += s;
		return pol;
	}

	/**
	 * @brief Scalar subtraction
	 * @param s [in] scalar to subtract.
	 * @return new polynomial after subtraction.
	 */
	const PolynomialND<Coef, Scalar> operator-(Scalar s) const
	{
		PolynomialND<Coef, Scalar> pol = *this;
		pol[0] -= s;
		return pol;
	}

	/**
	 * @brief Scalar multiplication
	 * @param s [in] scalar to multiply with.
	 * @return new polynomial after multiplication.
	 */
	const PolynomialND<Coef, Scalar> operator*(Scalar s) const
	{
		PolynomialND<Coef, Scalar> pol(order());
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
	const PolynomialND<Coef, Scalar> operator/(Scalar s) const
	{
		PolynomialND<Coef, Scalar> pol(order());
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
	PolynomialND<Coef, Scalar>& operator+=(Scalar s)
	{
		_coef[0] += s;
		return *this;
	}

	/**
	 * @brief Scalar subtraction
	 * @param s [in] scalar to subtract.
	 * @return same polynomial with coefficients changed.
	 */
	PolynomialND<Coef, Scalar>& operator-=(Scalar s)
	{
		_coef[0] -= s;
		return *this;
	}

	/**
	 * @brief Scalar multiplication
	 * @param s [in] the scalar to multiply with.
	 * @return reference to same polynomial with changed coefficients.
	 */
	PolynomialND<Coef, Scalar>& operator*=(Scalar s) {
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
	PolynomialND<Coef, Scalar>& operator/=(Scalar s) {
		for (std::size_t i = 0; i <= order(); i++) {
			_coef[i] /= s;
		}
		return *this;
	}

	/**
	 * @brief Scalar multiplication
	 * @param s [in] scalar to multiply with.
	 * @param p [in] polynomial to multiply with.
	 * @return new polynomial after multiplication.
	 */
	friend const PolynomialND<Coef, Scalar> operator*(Scalar s, const PolynomialND<Coef, Scalar>& p)
	{
		PolynomialND<Coef, Scalar> pol(p.order());
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
	const PolynomialND<Coef, Scalar> operator-(const PolynomialND<Coef, Scalar>& b) const
	{
		std::size_t ord = std::max<std::size_t>(order(),b.order());
		PolynomialND<Coef, Scalar> pol(ord);
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
	PolynomialND<Coef, Scalar>& operator-=(const PolynomialND<Coef, Scalar>& b) {
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
	const PolynomialND<Coef, Scalar> operator+(const PolynomialND<Coef, Scalar>& b) const
	{
		std::size_t ord = std::max<std::size_t>(order(),b.order());
		PolynomialND<Coef, Scalar> pol(ord);
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
	PolynomialND<Coef, Scalar>& operator+=(const PolynomialND<Coef, Scalar>& b) {
		std::size_t ord = std::max<std::size_t>(order(),b.order());
		if (ord > order())
			increaseOrder(ord-order());
		for (std::size_t i = 0; i <= b.order(); i++) {
			_coef[i] += b[i];
		}
		return *this;
	}

	/**
	 * @brief Polynomial multiplication.
	 *
	 * A convolution of the coefficients is used.
	 * Notice that more efficient algorithms exist for polynomials with scalar coefficients.
	 *
	 * @param b [in] polynomial to multiply. Post-multiplication is used - the dimensions must match.
	 * @return new polynomial after multiplication.
	 */
	template<typename OutCoef, typename Coef2 = Coef>
	PolynomialND<OutCoef, Scalar> multiply(const PolynomialND<Coef2, Scalar>& b) const {
		const std::size_t ord = order()+b.order();
		PolynomialND<OutCoef, Scalar> pol(ord);
		for (std::size_t k = 0; k <= ord; k++) {
			pol[ord-k] *= 0;
			for (std::size_t j = (k < b.order())? 0 : k-b.order(); j <= std::min(k,order()); j++) {
				pol[ord-k] += _coef[order()-j]*b[b.order()-(k-j)];
			}
		}
		return pol;
	}

	/**
	 * @brief Multiply with a coefficient.
	 *
	 * Each coefficient is post-multiplied with the given coefficient.
	 *
	 * @param b [in] coefficient to multiply with. Post-multiplication is used - the dimensions must match.
	 * @return new polynomial after multiplication.
	 */
	template<typename OutCoef, typename Coef2 = Coef>
	PolynomialND<OutCoef, Scalar> multiply(const Coef2& b) const {
		PolynomialND<OutCoef, Scalar> pol(order());
		for (std::size_t i = 0; i <= order(); i++) {
			pol[i] = _coef[i]*b;
		}
		return pol;
	}

	/**
	 * @brief Assignment.
	 * @param b [in] the polynomial to take coefficients from.
	 * @return true if equal, false if not.
	 */
	void operator=(const PolynomialND<Coef, Scalar>& b) {
		_coef.resize(b.order()+1);
		for(size_t i=0;i<=b.order();i++)
			_coef[i] = b[i];
	}

	///@}

	/**
	 * @brief Negate coefficients.
	 * @return new polynomial with coefficients negated.
	 */
	const PolynomialND<Coef, Scalar> operator-() const
	{
		PolynomialND<Coef, Scalar> pol(order());
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
	friend std::ostream& operator<<(std::ostream& out, const PolynomialND<Coef, Scalar>& p)
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
	bool operator==(const PolynomialND<Coef, Scalar>& b) const {
		for(size_t i=0;i<=order();i++)
			if(_coef[i]!=b[i])
				return false;
		return true;
	}

protected:
	//! @brief The coefficient vector.
	std::vector<Coef> _coef;
};

/**
 * @brief Multiply 3D polynomial matrix with 3D polynomial vector.
 * @param A [in] the matrix expression.
 * @param b [in] the vector expression.
 * @return a 3D polynomial vector.
 */
PolynomialND<Eigen::Vector3d> operator*(const PolynomialND<Eigen::Matrix3d>& A, const PolynomialND<Eigen::Vector3d>& b);

/**
 * @brief Multiply 3D polynomial vector with 3D polynomial matrix.
 * @param a [in] the vector expression.
 * @param A [in] the matrix expression.
 * @return a 3D polynomial vector.
 */
PolynomialND<Eigen::Matrix<double,1,3> > operator*(const PolynomialND<Eigen::Matrix<double,1,3> >& a, const PolynomialND<Eigen::Matrix3d>& A);

//! @copydoc operator*(const PolynomialND<Eigen::Matrix3d>&, const PolynomialND<Eigen::Vector3d>&)
PolynomialND<Eigen::Vector3d> operator*(const PolynomialND<Eigen::Matrix3d>& A, const Eigen::Vector3d& b);

//! @copydoc operator*(const PolynomialND<Eigen::Matrix<double,1,3> >&, const PolynomialND<Eigen::Matrix3d>&)
PolynomialND<Eigen::Matrix<double,1,3> > operator*(const PolynomialND<Eigen::Matrix<double,1,3> >& a, const Eigen::Matrix3d& A);

//! @copydoc operator*(const PolynomialND<Eigen::Matrix3d>&, const PolynomialND<Eigen::Vector3d>&)
PolynomialND<Eigen::Vector3f,float> operator*(const PolynomialND<Eigen::Matrix3f,float>& A, const PolynomialND<Eigen::Vector3f,float>& b);

//! @copydoc operator*(const PolynomialND<Eigen::Matrix<double,1,3> >&, const PolynomialND<Eigen::Matrix3d>&)
PolynomialND<Eigen::Matrix<float,1,3>,float> operator*(const PolynomialND<Eigen::Matrix<float,1,3>,float>& a, const PolynomialND<Eigen::Matrix3f,float>& A);

//! @copydoc operator*(const PolynomialND<Eigen::Matrix3d>&, const Eigen::Vector3d&)
PolynomialND<Eigen::Vector3f,float> operator*(const PolynomialND<Eigen::Matrix3f,float>& A, const Eigen::Vector3f& b);

//! @copydoc operator*(const PolynomialND<Eigen::Matrix<double,1,3> >&, const Eigen::Matrix3d&)
PolynomialND<Eigen::Matrix<float,1,3>,float> operator*(const PolynomialND<Eigen::Matrix<float,1,3>,float>& a, const Eigen::Matrix3f& A);

//! @}
} /* namespace math */
} /* namespace rw */

#endif /* RW_MATH_POLYNOMIALND_HPP_ */
