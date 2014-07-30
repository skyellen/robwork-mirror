/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#include "WorkCellJacobian.hpp"

#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {


WorkCellJacobian::WorkCellJacobian(WorkCellCalibration::Ptr calibration) {

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

	_compositeJointEncoderJacobian = rw::common::ownedPtr(new CompositeJacobian<JointEncoderJacobian>());
	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = calibration->getCompositeJointEncoderCalibration();
	for (int calibrationIndex = 0; calibrationIndex < compositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
		JointEncoderCalibration::Ptr jointCalibration = compositeJointCalibration->getCalibration(calibrationIndex);
		JointEncoderJacobian::Ptr jacobian = rw::common::ownedPtr(new JointEncoderJacobian(jointCalibration));
		_compositeJointEncoderJacobian->addJacobian(jacobian);
	}

	addJacobian(_compositeFixedFrameJacobian.cast<Jacobian>());
	addJacobian(_compositeLinkJacobian.cast<Jacobian>());
	addJacobian(_compositeJointEncoderJacobian.cast<Jacobian>());
}

WorkCellJacobian::~WorkCellJacobian() {

}



}
}
