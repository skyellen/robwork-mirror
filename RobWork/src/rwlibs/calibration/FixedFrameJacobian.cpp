/*
* FixedFrameJacobian.cpp
*
*  Created on: Nov 20, 2012
*      Author: bing
*/

#include "FixedFrameJacobian.hpp"

#include <rw/kinematics.hpp>


#include "DHLinkJacobian.hpp"

namespace rwlibs {
	namespace calibration {


		FixedFrameJacobian::FixedFrameJacobian(FixedFrameCalibration::Ptr calibration) :
			JacobianBase(calibration), _calibration(calibration), _fixedFrame(calibration->getFrame()) 
		{

		}

		FixedFrameJacobian::~FixedFrameJacobian() {

		}

		Eigen::MatrixXd FixedFrameJacobian::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
			const rw::kinematics::State& state) {
				const CalibrationParameterSet parameterSet = _calibration->getParameterSet();
				const Eigen::Affine3d correctionTransform = toEigen( _calibration->getCorrectionTransform() );
				
				const rw::kinematics::Frame* correctionFrame = _fixedFrame.get();
				//std::cout<<"Correction Frame = "<<_fixedFrame->getName()<<std::endl;
				//std::cout<<"Ref = "<<referenceFrame->getName()<<std::endl;
				Eigen::Affine3d refTcorrection = toEigen( rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), correctionFrame, state) );
				refTcorrection = refTcorrection * correctionTransform.inverse();
				Eigen::Affine3d correction2target = toEigen( rw::kinematics::Kinematics::frameTframe(correctionFrame, targetFrame.get(), state));
				Eigen::Matrix3d refRcorrection = refTcorrection.linear();
				Eigen::Vector3d preToEndTranslation = (refTcorrection * correctionTransform * correction2target).translation() - refTcorrection.translation();
				
				if (_fixedFrame == referenceFrame) {
					preToEndTranslation = -preToEndTranslation;
					refRcorrection *= -1;
				}
				
				const int columnCount = getColumnCount();
				Eigen::MatrixXd jacobianMatrix(6, columnCount);
				int columnIndex = 0;
				if (parameterSet(FixedFrameCalibration::PARAMETER_X).isEnabled()) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = refRcorrection.col(0);
					jacobianMatrix.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (parameterSet(FixedFrameCalibration::PARAMETER_Y).isEnabled()) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = refRcorrection.col(1);
					jacobianMatrix.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (parameterSet(FixedFrameCalibration::PARAMETER_Z).isEnabled()) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = refRcorrection.col(2);
					jacobianMatrix.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				
				double rollAngle = parameterSet(FixedFrameCalibration::PARAMETER_ROLL);
				Eigen::Matrix3d localRotation = refRcorrection * Eigen::AngleAxisd(rollAngle, Eigen::Vector3d::UnitZ());
				if (parameterSet(FixedFrameCalibration::PARAMETER_ROLL).isEnabled()) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = localRotation.col(2).cross(preToEndTranslation);
					jacobianMatrix.block<3, 1>(3, columnIndex) = localRotation.col(2);
					columnIndex++;
				}
				double pitchAngle = parameterSet(FixedFrameCalibration::PARAMETER_PITCH);
				localRotation = localRotation * Eigen::AngleAxisd(pitchAngle, Eigen::Vector3d::UnitY());
				if (parameterSet(FixedFrameCalibration::PARAMETER_PITCH).isEnabled()) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = localRotation.col(1).cross(preToEndTranslation);
					jacobianMatrix.block<3, 1>(3, columnIndex) = localRotation.col(1);
					columnIndex++;
				}
				double yawAngle = parameterSet(FixedFrameCalibration::PARAMETER_YAW);
				localRotation = localRotation * Eigen::AngleAxisd(yawAngle, Eigen::Vector3d::UnitX());
				if (parameterSet(FixedFrameCalibration::PARAMETER_YAW).isEnabled()) {
					jacobianMatrix.block<3, 1>(0, columnIndex) = localRotation.col(0).cross(preToEndTranslation);
					jacobianMatrix.block<3, 1>(3, columnIndex) = localRotation.col(0);
					columnIndex++;
				}

				RW_ASSERT(columnIndex == columnCount);

				return jacobianMatrix;
		}

	}
}
