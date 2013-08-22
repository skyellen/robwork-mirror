/*
 * FixedFrameJacobian.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "JacobianBase.hpp"
#include "FixedFrameCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/kinematics.hpp>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class FixedFrameJacobian: public JacobianBase {
public:
	typedef rw::common::Ptr<FixedFrameJacobian> Ptr;

	FixedFrameJacobian(FixedFrameCalibration::Ptr calibration);

	virtual ~FixedFrameJacobian();
	
protected:
	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

private:
	FixedFrameCalibration::Ptr _calibration;
	rw::kinematics::FixedFrame::Ptr _fixedFrame;	
};

}
}

#endif /* RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_ */
