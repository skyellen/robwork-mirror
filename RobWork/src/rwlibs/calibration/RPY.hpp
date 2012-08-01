/*
 * RPY.hpp
 *
 *  Created on: Oct 23, 2011
 *      Author: michael
 */

#ifndef RWLIBS_CALIBRATION_RPY_HPP
#define RWLIBS_CALIBRATION_RPY_HPP

#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Geometry>
#include <rw/common.hpp>

// http://stackoverflow.com/questions/652155/invalid-use-of-incomplete-type
namespace rwlibs {
namespace calibration {

template<typename _Scalar>
class RPY;
}
}

namespace Eigen {
namespace internal {
template<typename _Scalar> struct traits<rwlibs::calibration::RPY<_Scalar> > {
	typedef _Scalar Scalar;
};
}
}

namespace rwlibs {
namespace calibration {

template<typename _Scalar>
class RPY: public Eigen::RotationBase<RPY<_Scalar>, 3> {
public:
	RPY();

	RPY(const _Scalar& roll, const _Scalar& pitch, const _Scalar& yaw);

	template <typename Derived>
	explicit RPY(const Eigen::MatrixBase<Derived>& matrix, _Scalar epsilon = 1e-5);

	const _Scalar& roll() const;

	const _Scalar& pitch() const;

	const _Scalar& yaw() const;

	Eigen::Matrix<_Scalar, 3, 1> toVector() const;

	Eigen::Matrix<_Scalar, 3, 3> toRotationMatrix() const;

private:
	_Scalar roll_;
	_Scalar pitch_;
	_Scalar yaw_;
};

typedef RPY<float> RPYf;
typedef RPY<double> RPYd;

template<typename _Scalar>
RPY<_Scalar>::RPY() {
}

template<typename _Scalar>
RPY<_Scalar>::RPY(const _Scalar& roll, const _Scalar& pitch, const _Scalar& yaw) {
	roll_ = roll;
	pitch_ = pitch;
	yaw_ = yaw;
}

template<typename _Scalar>
template <typename Derived>
RPY<_Scalar>::RPY(const Eigen::MatrixBase<Derived>& matrix, _Scalar epsilon) {
	// TODO elaborate exceptions
	if (matrix.rows() != 3 || (matrix.cols() != 1 && matrix.cols() != 3))
		RW_THROW("RPY must be constructed from a 3x1 vector or 3x3 matrix.");

	if (matrix.cols() == 1) {
		roll_ = matrix(0);
		pitch_ = matrix(1);
		yaw_ = matrix(2);
	} else if (matrix.cols() == 3) {
		Eigen::Vector3d rpy = matrix.eulerAngles(2, 1, 0);
		roll_ = rpy(2);
		pitch_ = rpy(1);
		yaw_ = rpy(0);
		/*
		_Scalar sum = matrix(0, 0) * matrix(0, 0) + matrix(1, 0) * matrix(1, 0);

		const _Scalar cos_beta = sqrt(sum);
		const _Scalar sin_beta = -matrix(2, 0);

		// If beta == 90 deg or beta == -90 deg:
		if (fabs(cos_beta) < epsilon) {
			// If beta == -90 deg:
			if (sin_beta < 0) {
				yaw_ = 0;
				pitch_ = static_cast<_Scalar>(-M_PI / 2);
				roll_ = -atan2(matrix(0, 1), matrix(1, 1));
			}

			// If beta == 90 deg:
			else {
				yaw_ = 0;
				pitch_ = static_cast<_Scalar>(M_PI / 2);
				roll_ = atan2(matrix(0, 1), matrix(1, 1));
			}

		} else {
			pitch_ = static_cast<_Scalar>(atan2(sin_beta, cos_beta));
			yaw_ = static_cast<_Scalar>(atan2(matrix(1, 0), matrix(0, 0)));
			roll_ = static_cast<_Scalar>(atan2(matrix(2, 1), matrix(2, 2)));
		}*/
	}
}

template<typename _Scalar>
inline const _Scalar& RPY<_Scalar>::roll() const {
	return roll_;
}

template<typename _Scalar>
inline const _Scalar& RPY<_Scalar>::pitch() const {
	return pitch_;
}

template<typename _Scalar>
inline const _Scalar& RPY<_Scalar>::yaw() const {
	return yaw_;
}

template<typename _Scalar>
inline Eigen::Matrix<_Scalar, 3, 1> RPY<_Scalar>::toVector() const {
	return Eigen::Matrix<_Scalar, 3, 1>(roll_, pitch_, yaw_);
}

template<typename _Scalar>
inline Eigen::Matrix<_Scalar, 3, 3> RPY<_Scalar>::toRotationMatrix() const {
	return Eigen::Matrix<_Scalar, 3, 3>(Eigen::AngleAxis<_Scalar>(yaw_, Eigen::Matrix<_Scalar, 3, 1>::UnitZ()) * Eigen::AngleAxis<_Scalar>(pitch_, Eigen::Matrix<_Scalar, 3, 1>::UnitY()) * Eigen::AngleAxis<_Scalar>(roll_, Eigen::Matrix<_Scalar, 3, 1>::UnitX()));
}

}
}

#endif /* RWLIBS_CALIBRATION_RPY_HPP */
