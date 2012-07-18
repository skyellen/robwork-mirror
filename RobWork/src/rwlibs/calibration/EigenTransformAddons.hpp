/*
 * EigenTransformAddons.hpp
 *
 *  Created on: Oct 23, 2011
 *      Author: michael
 */

#ifndef RWLIBS_CALIBRATION_EIGENTRANSFORMADDONS_HPP
#define RWLIBS_CALIBRATION_EIGENTRANSFORMADDONS_HPP

inline Transform(const rw::math::Transform3D<>& transform3d) {
	check_template_params();
	rw::math::Vector3D<Scalar> vector3d = transform3d.P();
	rw::math::Rotation3D<Scalar> rotation3d = transform3d.R();
	Matrix<double, 4, 4> matrix = Matrix<double, 4, 4>::Identity();
	matrix(0,3) = vector3d(0);
	matrix(1,3) = vector3d(1);
	matrix(2,3) = vector3d(2);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			matrix(i,j) = rotation3d(i,j);
	*this = matrix;
}

operator rw::math::Transform3D<>() const {
	rw::math::Transform3D<> rwTfm;

	for (int rowNo = 0; rowNo < this->matrix().rows()-1; rowNo++)
		for (int colNo = 0; colNo < this->matrix().cols(); colNo++)
			rwTfm(rowNo, colNo) = this->matrix()(rowNo, colNo);

	return rwTfm;
}

#endif /* RWLIBS_CALIBRATION_EIGENTRANSFORMADDONS_HPP */
