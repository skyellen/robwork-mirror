/*
* DHLinkJacobian.cpp
*
*  Created on: Nov 20, 2012
*      Author: bing
*/

#include "DHLinkJacobian.hpp"

namespace rwlibs {
	namespace calibration {
		DHLinkJacobian::DHLinkJacobian(DHLinkCalibration::Ptr calibration) : JacobianBase(calibration), _calibration(calibration), _joint(calibration->getJoint()) {
		}

		DHLinkJacobian::~DHLinkJacobian() {

		}

		Eigen::MatrixXd DHLinkJacobian::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
			const rw::kinematics::State& state) {
				const CalibrationParameterSet parameterSet = _calibration->getParameterSet();

				const Eigen::Affine3d tfmToPreLink = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), _joint->getParent(state), state);
				const Eigen::Affine3d tfmLink = _joint->getFixedTransform();
				const Eigen::Affine3d tfmToPostLink = tfmToPreLink * tfmLink;
				const Eigen::Affine3d tfmJoint = _joint->getJointTransform(state);
				const Eigen::Affine3d tfmPostJoint = rw::kinematics::Kinematics::frameTframe(_joint.get(), targetFrame.get(), state);
				const Eigen::Affine3d tfmToEnd = tfmToPostLink * tfmJoint * tfmPostJoint;

				const unsigned int columnCount = getColumnCount();
				Eigen::MatrixXd jacobian(6, columnCount);
				int columnIndex = 0;
				if (parameterSet(DHLinkCalibration::PARAMETER_A).isEnabled()) {
					jacobian.block<3, 1>(0, columnIndex) = tfmToPostLink.linear().col(0);
					jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (parameterSet(DHLinkCalibration::PARAMETER_B).isEnabled()) {
					jacobian.block<3, 1>(0, columnIndex) = tfmToPreLink.linear().col(1);
					jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (parameterSet(DHLinkCalibration::PARAMETER_D).isEnabled()) {
					jacobian.block<3, 1>(0, columnIndex) = tfmToPreLink.linear().col(2);
					jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (parameterSet(DHLinkCalibration::PARAMETER_ALPHA).isEnabled()) {
					const Eigen::Vector3d xAxisToPost = tfmToPostLink.linear().col(0);
					const Eigen::Vector3d tlPostToEnd = tfmToEnd.translation() - tfmToPostLink.translation();
					jacobian.block<3, 1>(0, columnIndex) = xAxisToPost.cross(tlPostToEnd);
					jacobian.block<3, 1>(3, columnIndex) = xAxisToPost;
					columnIndex++;
				}
				if (parameterSet(DHLinkCalibration::PARAMETER_BETA).isEnabled()) {
					const Eigen::Vector3d yAxisToPre = tfmToPreLink.linear().col(1);
					const Eigen::Vector3d tlPreToEnd = tfmToEnd.translation() - tfmToPreLink.translation();
					jacobian.block<3, 1>(0, columnIndex) = yAxisToPre.cross(tlPreToEnd);
					jacobian.block<3, 1>(3, columnIndex) = yAxisToPre;
					columnIndex++;
				}
				if (parameterSet(DHLinkCalibration::PARAMETER_THETA).isEnabled()) {
					const Eigen::Vector3d zAxisToPre = tfmToPreLink.linear().col(2);
					const Eigen::Vector3d tlPreToEnd = tfmToEnd.translation() - tfmToPreLink.translation();
					jacobian.block<3, 1>(0, columnIndex) = zAxisToPre.cross(tlPreToEnd);
					jacobian.block<3, 1>(3, columnIndex) = zAxisToPre;
					columnIndex++;
				}

				RW_ASSERT(columnIndex == columnCount);

				return jacobian;
		}
	}
}
