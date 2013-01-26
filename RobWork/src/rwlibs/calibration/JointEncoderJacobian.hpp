/*
 * JointEncoderJacobian.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_JOINTENCODERJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_JOINTENCODERJACOBIAN_HPP_

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "JointEncoderCalibration.hpp"
#include "JacobianBase.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class JointEncoderJacobian: public JacobianBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<JointEncoderJacobian> Ptr;

	JointEncoderJacobian(JointEncoderCalibration::Ptr calibration);

	virtual ~JointEncoderJacobian();

protected:
	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

private:
	JointEncoderCalibration::Ptr _calibration;
	rw::models::JointDevice::Ptr _device;
	rw::models::Joint::Ptr _joint;
	int _jointIndex;
};

}
}

#endif /* RWLIBS_CALIBRATION_JOINTENCODERJACOBIAN_HPP_ */
