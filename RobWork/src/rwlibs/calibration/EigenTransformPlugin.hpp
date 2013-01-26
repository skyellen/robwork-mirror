/*
 * EigenTransformAddons.hpp
 *
 *  Created on: Oct 23, 2011
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_EIGENTRANSFORMADDONS_HPP
#define RWLIBS_CALIBRATION_EIGENTRANSFORMADDONS_HPP

inline Transform(const rw::math::Transform3D<>& transform3d) {
	check_template_params();
	rw::math::Vector3D<Scalar> vector3d = transform3d.P();
	rw::math::Rotation3D<Scalar> rotation3d = transform3d.R();
	this->setIdentity();
	this->translation() << vector3d(0), vector3d(1), vector3d(2);
	for (int rowIndex = 0; rowIndex < 3; rowIndex++)
		for (int colIndex = 0; colIndex < 3; colIndex++)
			this->linear()(rowIndex, colIndex) = rotation3d(rowIndex, colIndex);
}

operator rw::math::Transform3D<>() const {
	rw::math::Transform3D<> rwTfm;

	for (int rowIndex = 0; rowIndex < this->matrix().rows() - 1; rowIndex++)
		for (int colIndex = 0; colIndex < this->matrix().cols(); colIndex++)
			rwTfm(rowIndex, colIndex) = this->matrix()(rowIndex, colIndex);

	return rwTfm;
}

#endif /* RWLIBS_CALIBRATION_EIGENTRANSFORMADDONS_HPP */
