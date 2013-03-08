/*
 * DHLinkJacobian.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DHLINKJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_DHLINKJACOBIAN_HPP_

#include <rw/math.hpp>
//#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "DHLinkCalibration.hpp"
#include "JacobianBase.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/models/DHParameterSet.hpp>

namespace rwlibs {
namespace calibration {

Eigen::Affine3d toEigen(const rw::math::Transform3D<>& t3d);

class DHLinkJacobian: public JacobianBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<DHLinkJacobian> Ptr;

	DHLinkJacobian(DHLinkCalibration::Ptr calibration);

	virtual ~DHLinkJacobian();

protected:
	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

private:
	DHLinkCalibration::Ptr _calibration;
	rw::models::Joint::Ptr _joint;
};

}
}

#endif /* RWLIBS_CALIBRATION_DHLINKJACOBIAN_HPP_ */
