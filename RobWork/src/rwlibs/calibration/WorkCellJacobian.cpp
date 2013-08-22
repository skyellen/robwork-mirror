/*
 * WorkCellJacobian.cpp
 *
  */

#include "WorkCellJacobian.hpp"

#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

using namespace rwlibs::calibration;

WorkCellJacobian::WorkCellJacobian(WorkCellCalibration::Ptr calibration) {
//	_baseJacobian = rw::common::ownedPtr(new FixedFrameJacobian(calibration->getBaseCalibration()));
//	_endJacobian = rw::common::ownedPtr(new FixedFrameJacobian(calibration->getEndCalibration()));

	_compositeFixedFrameJacobian = rw::common::ownedPtr(new CompositeJacobian<FixedFrameJacobian>());
	CompositeCalibration<FixedFrameCalibration>::Ptr compositeFixedFrameCalibration = calibration->getFixedFrameCalibrations();
	for (int calibrationIndex = 0; calibrationIndex < compositeFixedFrameCalibration->getCalibrationCount(); calibrationIndex++) {
		FixedFrameCalibration::Ptr ffCalibration = compositeFixedFrameCalibration->getCalibration(calibrationIndex);
		FixedFrameJacobian::Ptr jacobian = rw::common::ownedPtr(new FixedFrameJacobian(ffCalibration));
		_compositeFixedFrameJacobian->addJacobian(jacobian);
	}

	_compositeLinkJacobian = rw::common::ownedPtr(new CompositeJacobian<ParallelAxisDHJacobian>());
	CompositeCalibration<ParallelAxisDHCalibration>::Ptr compositeLinkCalibration = calibration->getCompositeLinkCalibration();
	for (int calibrationIndex = 0; calibrationIndex < compositeLinkCalibration->getCalibrationCount(); calibrationIndex++) {
		ParallelAxisDHCalibration::Ptr linkCalibration = compositeLinkCalibration->getCalibration(calibrationIndex);
		ParallelAxisDHJacobian::Ptr jacobian = rw::common::ownedPtr(new ParallelAxisDHJacobian(linkCalibration));
		_compositeLinkJacobian->addJacobian(jacobian);
	}

	_compositeJointJacobian = rw::common::ownedPtr(new CompositeJacobian<JointEncoderJacobian>());
	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = calibration->getCompositeJointCalibration();
	for (int calibrationIndex = 0; calibrationIndex < compositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
		JointEncoderCalibration::Ptr jointCalibration = compositeJointCalibration->getCalibration(calibrationIndex);
		JointEncoderJacobian::Ptr jacobian = rw::common::ownedPtr(new JointEncoderJacobian(jointCalibration));
		_compositeJointJacobian->addJacobian(jacobian);
	}

	//addJacobian(_baseJacobian.cast<Jacobian>());
	//addJacobian(_endJacobian.cast<Jacobian>());
	addJacobian(_compositeFixedFrameJacobian.cast<Jacobian>());
	addJacobian(_compositeLinkJacobian.cast<Jacobian>());
	addJacobian(_compositeJointJacobian.cast<Jacobian>());
}

WorkCellJacobian::~WorkCellJacobian() {

}

FixedFrameJacobian::Ptr WorkCellJacobian::getBaseJacobian() const {
	return _baseJacobian;
}

FixedFrameJacobian::Ptr WorkCellJacobian::getEndJacobian() const {
	return _endJacobian;
}

CompositeJacobian<ParallelAxisDHJacobian>::Ptr WorkCellJacobian::getCompositeLinkJacobian() const {
	return _compositeLinkJacobian;
}

CompositeJacobian<JointEncoderJacobian>::Ptr WorkCellJacobian::getCompositeJointJacobian() const {
	return _compositeJointJacobian;
}

}
}
