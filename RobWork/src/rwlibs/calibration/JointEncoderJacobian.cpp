/*
* JointEncoderJacobian.cpp
*
*  Created on: Dec 20, 2012
*      Author: bing
*/

#include "JointEncoderJacobian.hpp"

namespace rwlibs {
	namespace calibration {
		JointEncoderJacobian::JointEncoderJacobian(JointEncoderCalibration::Ptr calibration) : JacobianBase(calibration), _calibration(calibration), _device(calibration->getDevice()), _joint(calibration->getJoint()) {
			// Find joint number.
			const std::vector<rw::models::Joint*> joints = _device->getJoints();
			_jointIndex = (int)(std::find(joints.begin(), joints.end(), _joint.get()) - joints.begin());
		}

		JointEncoderJacobian::~JointEncoderJacobian() {

		}

		Eigen::MatrixXd JointEncoderJacobian::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
			const rw::kinematics::State& state) {
				// Prepare transformations.
				const Eigen::Affine3d tfmToPostJoint = toEigen( rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), _joint.get(), state) );
				const Eigen::Affine3d tfmPostJoint = toEigen( rw::kinematics::Kinematics::frameTframe(_joint.get(), targetFrame.get(), state));
				const Eigen::Affine3d tfmToEnd = tfmToPostJoint * tfmPostJoint;
				const Eigen::Vector3d posToEnd = tfmToEnd.translation() - tfmToPostJoint.translation();
				const Eigen::Vector3d jointAxis = tfmToPostJoint.linear().col(2);

				// Get joint value.
				const rw::math::Q q = _device->getQ(state);
				RW_ASSERT(_jointIndex < (int)q.size());
				const double qi = q[_jointIndex];

				const CalibrationParameterSet parameterSet = _calibration->getParameterSet();
				const std::vector<rw::math::Function<>::Ptr> correctionFunctions = _calibration->getCorrectionFunctions();

				// Setup jacobian.
				const int columnCount = getColumnCount();
				Eigen::MatrixXd jacobian(6, columnCount);
				int columnIndex = 0;
				for (int parameterIndex = 0; parameterIndex < parameterSet.getCount(); parameterIndex++) {
					if (parameterSet(parameterIndex).isEnabled()) {
						// Revolute joint implementation.
						jacobian.block<3, 1>(0, columnIndex) = correctionFunctions[parameterIndex]->f(qi) * jointAxis.cross(posToEnd);
						jacobian.block<3, 1>(3, columnIndex) = correctionFunctions[parameterIndex]->f(qi) * jointAxis;
						columnIndex++;
					}
				}
				RW_ASSERT(columnIndex == columnCount);

				return jacobian;
		}
	}
}
