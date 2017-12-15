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

#include "PolynomialND.hpp"

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
 * @brief Representation of an ordinary polynomial with scalar coefficients (that can be both real and complex).
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
class Polynomial: public PolynomialND<T, T> {
public:
	/**
	 * @brief Create polynomial with coefficients initialized to zero.
	 * @param order [in] the order of the polynomial.
	 */
	explicit Polynomial(std::size_t order):
		PolynomialND<T,T>(order)
	{
		PolynomialND<T,T>::_coef = std::vector<T>(order+1,0);
	}

	/**
	 * @brief Create polynomial from vector.
	 * @param coefficients [in] the coefficients ordered from lowest-order term to highest-order term.
	 */
	Polynomial(const std::vector<T> &coefficients):
		PolynomialND<T,T>(coefficients)
	{
	}

	/**
	 * @brief Create polynomial from other polynomial.
	 * @param p [in] the polynomial to copy.
	 */
	Polynomial(const Polynomial<T> &p):
		PolynomialND<T,T>(p)
	{
	}

	/**
	 * @brief Create polynomial from other polynomial.
	 * @param p [in] the polynomial to copy.
	 */
	Polynomial(const PolynomialND<T,T> &p):
		PolynomialND<T,T>(p)
	{
	}

	/**
	 * @brief Destructor
	 */
	virtual ~Polynomial() {
	}

	//! @copydoc PolynomialND<T,T>::evaluate
	T evaluate(const T &x) const {
		return PolynomialND<T,T>::evaluate(x);
	}

	/**
	 * @brief Evaluate the polynomial using Horner's Method.
	 *
	 * This function estimates the error of the result.
	 * For float or double types, the error type, ErrT, should be the same as the type T.
	 * For std::complex<float> or std::complex<double> types, the error type, ErrT, should be either float or double.
	 *
	 * @param x [in] the input parameter.
	 * @param err [out] estimate of the absolute error.
	 * @return the value \f$f(x)\f$.
	 * @note Error is the absolute size of the interval where \f$f(x)\f$ can be, assuming coefficients are exact.
	 */
	template<typename ErrT = T>
	T evaluate(const T &x, ErrT& err) const {
		const ErrT eps = std::numeric_limits<ErrT>::epsilon();
		// Horner's Method
		T res = PolynomialND<T,T>::_coef.back();
		ErrT errCoef = 0; // Error due to finite precision coefficients
		ErrT errX = 0; // Error due to finite precision x value
		ErrT errComb = 0; // Combinational error of error both in coefficients and x (magnitude very small - eps*eps)
		errCoef = abs(res);
		const ErrT dX = std::abs(x)*eps;
 		for (int i = static_cast<int>(PolynomialND<T,T>::_coef.size()-2); i >= 0; i--) {
 			errX = std::abs(res)*dX+errX*std::abs(x)+errX*dX;
			res = PolynomialND<T,T>::_coef[i]+res*x;
			errComb = errComb*std::abs(x)+errComb*dX+errCoef*eps*dX;
			errCoef = std::abs(PolynomialND<T,T>::_coef[i]) + std::abs(x)*errCoef;
		}
 		errCoef *= eps;
 		err = errCoef+errX+errComb;
		return res;
	}

	//! @copydoc PolynomialND<T,T>::evaluateDerivatives
	std::vector<T> evaluateDerivatives(const T &x, std::size_t n = 1) const {
		return PolynomialND<T,T>::evaluateDerivatives(x,n);
	}

	/**
	 * @brief Evaluate the first \b n derivatives of the polynomial using Horner's Method.
	 *
	 * This function estimates the error of the result.
	 * For float or double types, the error type, ErrT, should be the same as the type T.
	 * For std::complex<float> or std::complex<double> types, the error type, ErrT, should be either float or double.
	 *
	 * @param x [in] the input parameter.
	 * @param err [out] estimate of the absolute errors.
	 * @param n [in] the number of derivatives to find (default is the first derivative only)
	 * @return the value \f$\dot{f}(x)\f$.
	 * @note Error is the absolute size of the interval where \f$f(x)\f$ can be, assuming coefficients are exact.
	 */
	template<typename ErrT = T>
	std::vector<T> evaluateDerivatives(const T &x, std::vector<ErrT>& err, std::size_t n = 1) const {
		const ErrT eps = std::numeric_limits<ErrT>::epsilon();
		// Horner's Method
		err.resize(n+1);
		for (std::size_t i = 0; i < err.size(); i++)
			err[i] = 0;
		std::vector<T> res(n+1,0);
		res[0] = PolynomialND<T,T>::_coef.back();
		err[0] = std::abs(res[0]);
		for (int i = static_cast<int>(PolynomialND<T,T>::_coef.size()-2); i >= 0; i--) {
			int minJ = static_cast<int>(std::min<std::size_t>(n,PolynomialND<T,T>::_coef.size()-1-i));
			for (int j = minJ; j > 0; j--) {
				res[j] = res[j-1]+res[j]*x;
				err[j] = std::abs(res[j-1])+std::abs(x)*err[j];
			}
			res[0] = PolynomialND<T,T>::_coef[i]+res[0]*x;
			err[0] = std::abs(PolynomialND<T,T>::_coef[i])+std::abs(x)*err[0];
		}
		T k = 1;
		for (std::size_t i = 2; i <= n; i++) {
			const T kInit = k;
			for (std::size_t j = 0; j < i-1; j++) {
				k += kInit;
			}
			res[i] *= k;
			err[i] *= std::abs(k);
		}
		for (std::size_t i = 0; i <= n; i++) {
			err[i] *= eps;
		}
		return res;
	}

	//! @copydoc PolynomialND<T,T>::deflate
	Polynomial<T> deflate(const T &x) const {
		// Horner Method
		std::size_t no = PolynomialND<T,T>::order()-1;
		Polynomial<T> res(no);
		res[no] = PolynomialND<T,T>::_coef.back();
		for (int i = (int)no-1; i >= 0; i--) {
			res[i] = x*res[i+1]+PolynomialND<T,T>::_coef[i+1];
		}
		return res;
	}

	//! @copydoc PolynomialND<T,T>::derivative
	Polynomial<T> derivative(std::size_t n = 1) const {
		if (n == 0)
			return *this;
		std::size_t no = PolynomialND<T,T>::order()-1;
		Polynomial<T> der(no);
		T factor = 0;
		for (std::size_t i = 1; i <= PolynomialND<T, T>::order(); i++) {
			factor += 1;
			der[i - 1] = PolynomialND<T, T>::_coef[i] * factor;
		}
		return der.derivative(n-1);
	}

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
		Polynomial<T> pol(PolynomialND<T,T>::order());
		for (std::size_t i = 0; i <= PolynomialND<T,T>::order(); i++) {
			pol[i] = PolynomialND<T,T>::_coef[i]*s;
		}
		return pol;
	}

	/**
	 * @brief Polynomial multiplication
	 *
	 * This multiplication functions uses a convolution of the coefficients.
	 * More efficient implementations are possible.
	 *
	 * @param polynomial [in] polynomial to multiply with.
	 * @return new polynomial after multiplication.
	 */
	const Polynomial<T> operator*(const Polynomial<T>& polynomial) const {
		return PolynomialND<T,T>::template multiply<T,T>(polynomial);
	}

	/**
	 * @brief Multiply polynomial with scalar coefficients with a 3D polynomial vector.
	 * @param p [in] polynomial with scalar coefficients.
	 * @param polynomial [in] polynomial vector.
	 * @return a 3D polynomial vector.
	 */
	friend PolynomialND<Eigen::Matrix<T,3,1>,T> operator*(const Polynomial<T>& p, const PolynomialND<Eigen::Matrix<T,3,1>,T>& polynomial) {
		return p.template multiply<Eigen::Matrix<T,3,1>,Eigen::Matrix<T,3,1> >(polynomial);
	}

	//! @copydoc operator*(const Polynomial<T>&, const PolynomialND<Eigen::Matrix<T,3,1>,T>&)
	friend PolynomialND<Eigen::Matrix<T,1,3>,T> operator*(const Polynomial<T>& p, const PolynomialND<Eigen::Matrix<T,1,3>,T>& polynomial) {
		return p.template multiply<Eigen::Matrix<T,1,3>,Eigen::Matrix<T,1,3> >(polynomial);
	}

	/**
	 * @brief Multiply polynomial with scalar coefficients with a 3D polynomial matrix.
	 * @param p [in] polynomial with scalar coefficients.
	 * @param polynomial [in] polynomial matrix.
	 * @return a 3D polynomial matrix.
	 */
	friend PolynomialND<Eigen::Matrix<T,3,3>,T> operator*(const Polynomial<T>& p, const PolynomialND<Eigen::Matrix<T,3,3>,T>& polynomial) {
		return p.template multiply<Eigen::Matrix<T,3,3>,Eigen::Matrix<T,3,3> >(polynomial);
	}

	/**
	 * @brief Multiply polynomial with scalar coefficients with a vector.
	 * @param p [in] polynomial with scalar coefficients.
	 * @param a [in] vector to multiply with.
	 * @return a 3D polynomial vector.
	 */
	friend PolynomialND<Eigen::Matrix<T,3,1>,T> operator*(const Polynomial<T>& p, const Eigen::Matrix<T,3,1>& a) {
		return p.template multiply<Eigen::Matrix<T,3,1>,Eigen::Matrix<T,3,1> >(a);
	}

	//! @copydoc operator*(const Polynomial<T>&, const Eigen::Matrix<T,3,1>&)
	friend PolynomialND<Eigen::Matrix<T,1,3>,T> operator*(const Polynomial<T>& p, const Eigen::Matrix<T,1,3>& a) {
		return p.template multiply<Eigen::Matrix<T,1,3>,Eigen::Matrix<T,1,3> >(a);
	}

	/**
	 * @brief Multiply polynomial with scalar coefficients with a matrix.
	 * @param p [in] polynomial with scalar coefficients.
	 * @param A [in] matrix to multiply with.
	 * @return a 3D polynomial matrix.
	 */
	friend PolynomialND<Eigen::Matrix<T,3,3>,T> operator*(const Polynomial<T>& p, const Eigen::Matrix<T,3,3>& A) {
		return p.template multiply<Eigen::Matrix<T,3,3>,Eigen::Matrix<T,3,3> >(A);
	}

	/**
	 * @brief Scalar division
	 * @param s [in] scalar to divide with.
	 * @return new polynomial after division.
	 */
	const Polynomial<T> operator/(T s) const
	{
		Polynomial<T> pol(PolynomialND<T,T>::order());
		for (std::size_t i = 0; i <= PolynomialND<T,T>::order(); i++) {
			pol[i] = PolynomialND<T,T>::_coef[i]/s;
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
		PolynomialND<T,T>::_coef[0] += s;
		return *this;
	}

	/**
	 * @brief Scalar subtraction
	 * @param s [in] scalar to subtract.
	 * @return same polynomial with coefficients changed.
	 */
	Polynomial<T>& operator-=(T s)
	{
		PolynomialND<T,T>::_coef[0] -= s;
		return *this;
	}

	/**
	 * @brief Scalar multiplication
	 * @param s [in] the scalar to multiply with.
	 * @return reference to same polynomial with changed coefficients.
	 */
	Polynomial<T>& operator*=(T s) {
		for (std::size_t i = 0; i <= PolynomialND<T,T>::order(); i++) {
			PolynomialND<T,T>::_coef[i] *= s;
		}
		return *this;
	}

	/**
	 * @brief Scalar division
	 * @param s [in] the scalar to divide with.
	 * @return reference to same polynomial with changed coefficients.
	 */
	Polynomial<T>& operator/=(T s) {
		for (std::size_t i = 0; i <= PolynomialND<T,T>::order(); i++) {
			PolynomialND<T,T>::_coef[i] /= s;
		}
		return *this;
	}

	/**
	 * @brief Scalar multiplication
	 * @param s [in] scalar to multiply with.
	 * @param p [in] polynomial to multiply with.
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
		std::size_t ord = std::max<std::size_t>(PolynomialND<T,T>::order(),b.order());
		Polynomial<T> pol(ord);
		for (std::size_t i = 0; i <= PolynomialND<T,T>::order(); i++) {
			pol[i] = PolynomialND<T,T>::_coef[i];
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
		std::size_t ord = std::max<std::size_t>(PolynomialND<T,T>::order(),b.order());
		if (ord > PolynomialND<T,T>::order())
			PolynomialND<T,T>::increaseOrder(ord-PolynomialND<T,T>::order());
		for (std::size_t i = 0; i <= b.order(); i++) {
			PolynomialND<T,T>::_coef[i] -= b[i];
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
		std::size_t ord = std::max<std::size_t>(PolynomialND<T,T>::order(),b.order());
		Polynomial<T> pol(ord);
		for (std::size_t i = 0; i <= PolynomialND<T,T>::order(); i++) {
			pol[i] = PolynomialND<T,T>::_coef[i];
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
		std::size_t ord = std::max<std::size_t>(PolynomialND<T,T>::order(),b.order());
		if (ord > PolynomialND<T,T>::order())
			PolynomialND<T,T>::increaseOrder(ord-PolynomialND<T,T>::order());
		for (std::size_t i = 0; i <= b.order(); i++) {
			PolynomialND<T,T>::_coef[i] += b[i];
		}
		return *this;
	}

	/**
	 * @brief Assignment.
	 * @param b [in] the polynomial to take coefficients from.
	 * @return true if equal, false if not.
	 */
	void operator=(const Polynomial<T>& b) {
		PolynomialND<T,T>::_coef.resize(b.order()+1);
		for(size_t i=0;i<=b.order();i++)
			PolynomialND<T,T>::_coef[i] = b[i];
	}

	///@}

	/**
	 * @brief Negate coefficients.
	 * @return new polynomial with coefficients negated.
	 */
	const Polynomial<T> operator-() const
	{
		Polynomial<T> pol(PolynomialND<T,T>::order());
		for (std::size_t i = 0; i <= PolynomialND<T,T>::order(); i++) {
			pol[i] = -PolynomialND<T,T>::_coef[i];
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
		for(size_t i=0;i<=PolynomialND<T,T>::order();i++)
			if(PolynomialND<T,T>::_coef[i]!=b[i])
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
		Polynomial<Q> pol(PolynomialND<T,T>::order());
		for (std::size_t i = 0; i <= PolynomialND<T,T>::order(); i++) {
			pol[i] = static_cast<Q>(PolynomialND<T,T>::_coef[i]);
		}
		return pol;
	}
};

/**
 * @brief Multiply 3D polynomial vector with 3D polynomial vector.
 * @param a [in] first polynomial vector (row vector).
 * @param b [in] second polynomial vector (column vector).
 * @return a polynomial with scalar coefficients.
 */
Polynomial<> operator*(const PolynomialND<Eigen::Matrix<double,1,3> >& a, const PolynomialND<Eigen::Matrix<double,3,1> >& b);

/**
 * @brief Multiply 3D polynomial vector with a polynomial with scalar coefficients.
 * @param polynomial [in] the polynomial vector.
 * @param p [in] polynomial with scalar coefficients.
 * @return a 3D polynomial vector.
 */
PolynomialND<Eigen::Vector3d> operator*(const PolynomialND<Eigen::Vector3d>& polynomial, const Polynomial<>& p);

//! @copydoc operator*(const PolynomialND<Eigen::Vector3d>&, const Polynomial<>&)
PolynomialND<Eigen::Matrix<double,1,3> > operator*(const PolynomialND<Eigen::Matrix<double,1,3> >& polynomial, const Polynomial<>& p);

/**
 * @brief Multiply 3D polynomial matrix with a polynomial with scalar coefficients.
 * @param polynomial [in] the polynomial matrix.
 * @param p [in] polynomial with scalar coefficients.
 * @return a 3D polynomial matrix.
 */
PolynomialND<Eigen::Matrix3d> operator*(const PolynomialND<Eigen::Matrix3d >& polynomial, const Polynomial<>& p);

//! @copydoc operator*(const PolynomialND<Eigen::Matrix<double,1,3> >&, const PolynomialND<Eigen::Matrix<double,3,1> >&)
Polynomial<float> operator*(const PolynomialND<Eigen::Matrix<float,1,3>,float>& a, const PolynomialND<Eigen::Matrix<float,3,1>,float>& b);

//! @copydoc operator*(const PolynomialND<Eigen::Vector3d>&, const Polynomial<>&)
PolynomialND<Eigen::Vector3f,float> operator*(const PolynomialND<Eigen::Vector3f,float>& polynomial, const Polynomial<float>& p);

//! @copydoc operator*(const PolynomialND<Eigen::Matrix<double,1,3> >&, const Polynomial<>&)
PolynomialND<Eigen::Matrix<float,1,3>,float> operator*(const PolynomialND<Eigen::Matrix<float,1,3>,float>& polynomial, const Polynomial<float>& p);

//! @copydoc operator*(const PolynomialND<Eigen::Matrix3d >&, const Polynomial<>&)
PolynomialND<Eigen::Matrix3f,float> operator*(const PolynomialND<Eigen::Matrix3f,float>& polynomial, const Polynomial<float>& p);

//! @}
} /* namespace math */
} /* namespace rw */

namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
	/**
	 * @copydoc rw::common::serialization::write
	 * @relatedalso rw::math::Polynomial
	 */
	template<> void write(const rw::math::Polynomial<double>& sobject, rw::common::OutputArchive& oarchive, const std::string& id);

	/**
	 * @copydoc rw::common::serialization::write
	 * @relatedalso rw::math::Polynomial
	 */
	template<> void write(const rw::math::Polynomial<float>& sobject, rw::common::OutputArchive& oarchive, const std::string& id);

	/**
	 * @copydoc rw::common::serialization::read
	 * @relatedalso rw::math::Polynomial
	 */
	template<> void read(rw::math::Polynomial<double>& sobject, rw::common::InputArchive& iarchive, const std::string& id);


	/**
	 * @copydoc rw::common::serialization::read
	 * @relatedalso rw::math::Polynomial
	 */
	template<> void read(rw::math::Polynomial<float>& sobject, rw::common::InputArchive& iarchive, const std::string& id);
}}} // end namespaces

#endif /* RW_MATH_POLYNOMIAL_HPP_ */
