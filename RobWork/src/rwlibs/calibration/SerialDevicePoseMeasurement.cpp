/*
 * SerialDevicePoseMeasurement.cpp
 *
 *  Created on: Apr 14, 2012
 *      Author: bing
 */

#include "SerialDevicePoseMeasurement.hpp"

#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement() : _hasCovarianceMatrix(false) {

}

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform) :
	_q(q), _transform(transform), _hasCovarianceMatrix(false) {

}

SerialDevicePoseMeasurement::SerialDevicePoseMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::Matrix<double, 6, 6>& covarianceMatrix) :
	_q(q), _transform(transform), _covarianceMatrix(covarianceMatrix), _hasCovarianceMatrix(true) {

}

SerialDevicePoseMeasurement::~SerialDevicePoseMeasurement() {

}

const rw::math::Q& SerialDevicePoseMeasurement::getQ() const {
	return _q;
}

const rw::math::Transform3D<>& SerialDevicePoseMeasurement::getTransform() const {
	return _transform;
}

const Eigen::Matrix<double, 6, 6>& SerialDevicePoseMeasurement::getCovarianceMatrix() const {
	return _covarianceMatrix;
}

bool SerialDevicePoseMeasurement::hasCovarianceMatrix() const {
	return _hasCovarianceMatrix;
}

}
}
