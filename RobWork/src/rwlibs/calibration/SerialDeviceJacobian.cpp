/*
 * SerialDeviceJacobian.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#include "SerialDeviceJacobian.hpp"

#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

using namespace rwlibs::calibration;

SerialDeviceJacobian::SerialDeviceJacobian(SerialDeviceCalibration::Ptr calibration) {
	_baseJacobian = rw::common::ownedPtr(new FixedFrameJacobian(calibration->getBaseCalibration()));
	_endJacobian = rw::common::ownedPtr(new FixedFrameJacobian(calibration->getEndCalibration()));
	
	_compositeLinkJacobian = rw::common::ownedPtr(new CompositeJacobian<DHLinkJacobian>());
	CompositeCalibration<DHLinkCalibration>::Ptr compositeLinkCalibration = calibration->getCompositeLinkCalibration();
	for (int calibrationIndex = 0; calibrationIndex < compositeLinkCalibration->getCalibrationCount(); calibrationIndex++) {
		DHLinkCalibration::Ptr linkCalibration = compositeLinkCalibration->getCalibration(calibrationIndex);
		DHLinkJacobian::Ptr jacobian = rw::common::ownedPtr(new DHLinkJacobian(linkCalibration));
		_compositeLinkJacobian->addJacobian(jacobian);
	}

	_compositeJointJacobian = rw::common::ownedPtr(new CompositeJacobian<JointEncoderJacobian>());
	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = calibration->getCompositeJointCalibration();
	for (int calibrationIndex = 0; calibrationIndex < compositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
		JointEncoderCalibration::Ptr jointCalibration = compositeJointCalibration->getCalibration(calibrationIndex);
		JointEncoderJacobian::Ptr jacobian = rw::common::ownedPtr(new JointEncoderJacobian(jointCalibration));
		_compositeJointJacobian->addJacobian(jacobian);
	}

	addJacobian(_baseJacobian.cast<Jacobian>());
	addJacobian(_endJacobian.cast<Jacobian>());
	addJacobian(_compositeLinkJacobian.cast<Jacobian>());
	addJacobian(_compositeJointJacobian.cast<Jacobian>());
}

SerialDeviceJacobian::~SerialDeviceJacobian() {

}

FixedFrameJacobian::Ptr SerialDeviceJacobian::getBaseJacobian() const {
	return _baseJacobian;
}

FixedFrameJacobian::Ptr SerialDeviceJacobian::getEndJacobian() const {
	return _endJacobian;
}

CompositeJacobian<DHLinkJacobian>::Ptr SerialDeviceJacobian::getCompositeLinkJacobian() const {
	return _compositeLinkJacobian;
}

CompositeJacobian<JointEncoderJacobian>::Ptr SerialDeviceJacobian::getCompositeJointJacobian() const {
	return _compositeJointJacobian;
}

}
}
