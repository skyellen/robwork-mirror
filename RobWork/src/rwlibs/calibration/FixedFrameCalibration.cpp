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

#include "FixedFrameCalibration.hpp"

#include <rw/math/RPY.hpp>

using namespace rwlibs::calibration;

FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame) :
	 _frame(frame)		
{
	setCorrectedTransform(frame->getFixedTransform());
	if (frame.isNull())
		RW_THROW("Unable to construct FixedFrameCalibration for a frame which is NULL");
}

FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, const rw::math::Transform3D<>& correctedTransform) :
	_frame(frame)
{
	if (frame.isNull())
		RW_THROW("Unable to construct FixedFrameCalibration for a frame which is NULL");

	setCorrectedTransform(correctedTransform);
}

FixedFrameCalibration::~FixedFrameCalibration() {

}

rw::kinematics::FixedFrame::Ptr FixedFrameCalibration::getFrame() const {
	return _frame;
}



rw::math::Transform3D<> FixedFrameCalibration::getCorrectedTransform() const {
	return _correctedTransform;
}

void FixedFrameCalibration::setCorrectedTransform(const rw::math::Transform3D<>& transform) {
	_correctedTransform = transform;
} 

void FixedFrameCalibration::doApply() {
	RW_ASSERT(!_frame.isNull());
	_originalTransform = _frame->getFixedTransform();
	_frame->setTransform(_correctedTransform);
}

void FixedFrameCalibration::doRevert() {
	RW_ASSERT(!_frame.isNull());
	if (isApplied())
		_frame->setTransform(_originalTransform);		
}
  
