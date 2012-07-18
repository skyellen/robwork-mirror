/*
 * Pose6D.hpp
 *
 *  Created on: Oct 28, 2011
 *      Author: michael
 */

#ifndef RWLIBS_CALIBRATION_POSE_HPP
#define RWLIBS_CALIBRATION_POSE_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformAddons.hpp"

#include "RPY.hpp"

#include <Eigen/Core>

#include <iostream>

namespace rwlibs {
namespace calibration {

template<typename Scalar = double>
class Pose6D: public Eigen::Matrix<Scalar, 6, 1> {
public:
	Pose6D();

	Pose6D(Scalar x, Scalar y, Scalar z, Scalar roll, Scalar pitch, Scalar yaw);

	explicit Pose6D(const Eigen::Affine3d& transform);

	template<typename OtherDerived>
	Pose6D(const Eigen::MatrixBase<OtherDerived>& other);

	virtual ~Pose6D();

	Scalar& roll();

	const Scalar& roll() const;

	Scalar& pitch();

	const Scalar& pitch() const;

	Scalar& yaw();

	const Scalar& yaw() const;

	Eigen::Matrix<Scalar, 3, 1> translation() const;

	RPY<Scalar> rotation() const;

	Eigen::Affine3d operator*(const Eigen::Affine3d& other) const;

	Pose6D<double> operator*(const Pose6D<double>& other) const;

	operator Eigen::Affine3d() const;

	Pose6D<Scalar> inverse() const;

	Eigen::Affine3d toTransform() const;

	static Pose6D<Scalar> Zero();
};

template<typename Scalar>
inline Pose6D<Scalar>::Pose6D() :
		Eigen::Matrix<Scalar, 6, 1>() {

}

template<typename Scalar>
inline Pose6D<Scalar>::Pose6D(Scalar x, Scalar y, Scalar z, Scalar roll,
		Scalar pitch, Scalar yaw) {
	*this << x, y, z, roll, pitch, yaw;
}

template<typename Scalar>
inline Pose6D<Scalar>::Pose6D(const Eigen::Affine3d& transform) {
	this->head(3) = transform.translation();
	RPY < Scalar > rpy(transform.rotation().matrix());
	this->operator()(3) = rpy.roll();
	this->operator()(4) = rpy.pitch();
	this->operator()(5) = rpy.yaw();
}

template<typename Scalar>
Pose6D<Scalar>::~Pose6D() {

}

template<typename Scalar>
template<typename OtherDerived>
inline Pose6D<Scalar>::Pose6D(const Eigen::MatrixBase<OtherDerived>& other) {
	this->Eigen::Matrix<Scalar, 6, 1>::operator=(other);
}

template<typename Scalar>
inline Scalar& Pose6D<Scalar>::roll() {
	return this->operator()(3);
}

template<typename Scalar>
inline const Scalar& Pose6D<Scalar>::roll() const {
	return this->operator()(3);
}

template<typename Scalar>
inline Scalar& Pose6D<Scalar>::pitch() {
	return this->operator()(4);
}

template<typename Scalar>
inline const Scalar& Pose6D<Scalar>::pitch() const {
	return this->operator()(4);
}

template<typename Scalar>
inline Scalar& Pose6D<Scalar>::yaw() {
	return this->operator()(5);
}

template<typename Scalar>
inline const Scalar& Pose6D<Scalar>::yaw() const {
	return this->operator()(5);
}

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1> Pose6D<Scalar>::translation() const {
	return this->head(3);
}

template<typename Scalar>
inline RPY<Scalar> Pose6D<Scalar>::rotation() const {
	return RPY<Scalar>(this->roll(), this->pitch(), this->yaw());
}

template<typename Scalar>
inline Pose6D<Scalar> Pose6D<Scalar>::inverse() const {
	return Pose6D<Scalar>(toTransform().inverse());
}

template<typename Scalar>
inline Eigen::Affine3d Pose6D<Scalar>::toTransform() const {
	Eigen::Affine3d transform;
	transform.translation() = this->translation();
	transform.linear() = this->rotation().toRotationMatrix();
	return transform;
}

template<typename Scalar>
inline Eigen::Affine3d Pose6D<Scalar>::operator*(
		const Eigen::Affine3d& other) const {
	return this->toTransform() * other;
}

template<typename Scalar>
inline Pose6D<double> Pose6D<Scalar>::operator*(
		const Pose6D<double>& other) const {
	return Pose6D<double>(this->toTransform() * other.toTransform());
}

template<typename Scalar>
inline Pose6D<Scalar>::operator Eigen::Affine3d() const {
	return this->toTransform();
}

template<typename Scalar>
inline Pose6D<Scalar> Pose6D<Scalar>::Zero() {
	return Eigen::Matrix<double, 6, 1>::Zero();
}

}
}

#endif /* RWLIBS_CALIBRATION_POSE_HPP */
