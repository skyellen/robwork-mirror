/*
* ParallelAxisDHJacobian.cpp
*
*  Created on: Nov 20, 2012

*/

#include "ParallelAxisDHJacobian.hpp"
#include "ParallelAxisDHCalibration.hpp"

#include <rw/kinematics/Kinematics.hpp>

namespace rwlibs {
	namespace calibration { 



		ParallelAxisDHJacobian::ParallelAxisDHJacobian(ParallelAxisDHCalibration::Ptr calibration) : JacobianBase(calibration), _calibration(calibration), _joint(calibration->getJoint()) {
		}

		ParallelAxisDHJacobian::~ParallelAxisDHJacobian() {

		}

		Eigen::MatrixXd ParallelAxisDHJacobian::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
			const rw::kinematics::State& state) {
				const CalibrationParameterSet parameterSet = _calibration->getParameterSet();
				//std::cout<<"Reference Frame = "<<referenceFrame->getName()<<" Joint Parent = "<<_joint->getParent(state)->getName()<<std::endl;
				//std::cout<<"Joint = "<<_joint->getName()<<std::endl;
				const Eigen::Affine3d tfmToPreLink = toEigen(rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), _joint->getParent(state), state));
				const Eigen::Affine3d tfmLink = toEigen(_joint->getFixedTransform());
				const Eigen::Affine3d tfmToPostLink = tfmToPreLink * tfmLink;
				const Eigen::Affine3d tfmJoint = toEigen(_joint->getJointTransform(state));
				const Eigen::Affine3d tfmPostJoint = toEigen(rw::kinematics::Kinematics::frameTframe(_joint.get(), targetFrame.get(), state) );
				const Eigen::Affine3d tfmToEnd = tfmToPostLink * tfmJoint * tfmPostJoint;

				const unsigned int columnCount = getColumnCount();
				Eigen::MatrixXd jacobian(6, columnCount);
				int columnIndex = 0;
				if (parameterSet(ParallelAxisDHCalibration::PARAMETER_A).isEnabled()) {
					jacobian.block<3, 1>(0, columnIndex) = tfmToPostLink.linear().col(0);
					jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (parameterSet(ParallelAxisDHCalibration::PARAMETER_B).isEnabled()) {
					jacobian.block<3, 1>(0, columnIndex) = tfmToPostLink.linear().col(1);
					jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (parameterSet(ParallelAxisDHCalibration::PARAMETER_D).isEnabled()) {
					jacobian.block<3, 1>(0, columnIndex) = tfmToPostLink.linear().col(2);
					jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (parameterSet(ParallelAxisDHCalibration::PARAMETER_ALPHA).isEnabled()) {
					const Eigen::Vector3d xAxisToPost = tfmToPostLink.linear().col(0);
					const Eigen::Vector3d tlPostToEnd = tfmToEnd.translation() - tfmToPostLink.translation();
					jacobian.block<3, 1>(0, columnIndex) = xAxisToPost.cross(tlPostToEnd);
					jacobian.block<3, 1>(3, columnIndex) = xAxisToPost;
					columnIndex++;
				}
				if (parameterSet(ParallelAxisDHCalibration::PARAMETER_BETA).isEnabled()) {
					const Eigen::Vector3d yAxisToPost = tfmToPostLink.linear().col(1);
					const Eigen::Vector3d tlPostToEnd = tfmToEnd.translation() - tfmToPostLink.translation();
					jacobian.block<3, 1>(0, columnIndex) = yAxisToPost.cross(tlPostToEnd);
					jacobian.block<3, 1>(3, columnIndex) = yAxisToPost;
					columnIndex++;
				}
				if (parameterSet(ParallelAxisDHCalibration::PARAMETER_THETA).isEnabled()) {
					const Eigen::Vector3d zAxisToPost = tfmToPostLink.linear().col(2);
					const Eigen::Vector3d tlPostToEnd = tfmToEnd.translation() - tfmToPostLink.translation();
					jacobian.block<3, 1>(0, columnIndex) = zAxisToPost.cross(tlPostToEnd);
					jacobian.block<3, 1>(3, columnIndex) = zAxisToPost;
					columnIndex++;
				}

				RW_ASSERT(columnIndex == (int)columnCount);

				return jacobian;
		}
	}
}
