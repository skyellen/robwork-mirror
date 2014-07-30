/*
* FixedFrameCalibration.cpp
*/

#include "FixedFrameCalibration.hpp"

#include <rw/kinematics.hpp>


namespace rwlibs {
	namespace calibration {
		//int FixedFrameCalibration::PARAMETER_X = 0;
		//int FixedFrameCalibration::PARAMETER_Y = 1;
		//int FixedFrameCalibration::PARAMETER_Z = 2;
		//int FixedFrameCalibration::PARAMETER_ROLL = 3;
		//int FixedFrameCalibration::PARAMETER_PITCH = 4;
		//int FixedFrameCalibration::PARAMETER_YAW = 5;

		FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame) :
			CalibrationBase(CalibrationParameterSet(6)), _frame(frame)		{
			if (frame.isNull())
				RW_THROW("Unable to construct FixedFrameCalibration for a frame which is NULL");
		}

		FixedFrameCalibration::FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, const rw::math::Transform3D<>& correctionTransform) :
			CalibrationBase(CalibrationParameterSet(6)), _frame(frame)
		{
			if (frame.isNull())
				RW_THROW("Unable to construct FixedFrameCalibration for a frame which is NULL");

			setCorrectionTransform(correctionTransform);
		}

		FixedFrameCalibration::~FixedFrameCalibration() {

		}

		rw::kinematics::FixedFrame::Ptr FixedFrameCalibration::getFrame() const {
			return _frame;
		}



		rw::math::Transform3D<> FixedFrameCalibration::getCorrectionTransform() const {
			CalibrationParameterSet parameterSet = getParameterSet();
			return rw::math::Transform3D<>(
				rw::math::Vector3D<>(
				parameterSet(PARAMETER_X).isEnabled() ? parameterSet(PARAMETER_X).getValue() : 0.0,
				parameterSet(PARAMETER_Y).isEnabled() ? parameterSet(PARAMETER_Y).getValue() : 0.0,
				parameterSet(PARAMETER_Z).isEnabled() ? parameterSet(PARAMETER_Z).getValue() : 0.0
				),
				rw::math::RPY<>(
				parameterSet(PARAMETER_ROLL).isEnabled() ? parameterSet(PARAMETER_ROLL).getValue() : 0.0,
				parameterSet(PARAMETER_PITCH).isEnabled() ? parameterSet(PARAMETER_PITCH).getValue() : 0.0,
				parameterSet(PARAMETER_YAW).isEnabled() ? parameterSet(PARAMETER_YAW).getValue() : 0.0
				).toRotation3D()
				);
		}

		void FixedFrameCalibration::setCorrectionTransform(const rw::math::Transform3D<>& transform) {
			CalibrationParameterSet parameterSet = getParameterSet();
			// HACK: Fix hack.
			parameterSet(PARAMETER_X) = transform.P()(0);
			parameterSet(PARAMETER_Y) = transform.P()(1);
			parameterSet(PARAMETER_Z) = transform.P()(2);
			rw::math::RPY<> rpy(transform.R());
			parameterSet(PARAMETER_ROLL) = rpy(0);
			parameterSet(PARAMETER_PITCH) = rpy(1);
			parameterSet(PARAMETER_YAW) = rpy(2);
			setParameterSet(parameterSet);
		} 

		void FixedFrameCalibration::doApply() {
			RW_ASSERT(!_frame.isNull());
			_originalTransform = _frame->getFixedTransform();
			const rw::math::Transform3D<> correctionTransform = getCorrectionTransform();
			const rw::math::Transform3D<> correctedTransform = _originalTransform * correctionTransform;
			_frame->setTransform(correctedTransform);
		}

		void FixedFrameCalibration::doRevert() {
			RW_ASSERT(!_frame.isNull());
			_frame->setTransform(_originalTransform);
		}
	}
}
  