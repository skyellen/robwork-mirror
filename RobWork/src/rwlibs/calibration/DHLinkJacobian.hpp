/*
 * DHLinkJacobian.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DHLINKJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_DHLINKJACOBIAN_HPP_

//#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "JacobianBase.hpp"
#include <Eigen/Geometry>

namespace rw { namespace models { class Joint; } }

namespace rwlibs {
namespace calibration {
class DHLinkCalibration;


class DHLinkJacobian: public JacobianBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<DHLinkJacobian> Ptr;

	DHLinkJacobian(rw::common::Ptr<DHLinkCalibration> calibration);

	virtual ~DHLinkJacobian();

protected:
	virtual Eigen::MatrixXd doComputeJacobian(rw::common::Ptr<rw::kinematics::Frame> referenceFrame, rw::common::Ptr<rw::kinematics::Frame> targetFrame, const rw::kinematics::State& state);

private:
	rw::common::Ptr<DHLinkCalibration> _calibration;
	rw::common::Ptr<rw::models::Joint> _joint;
};

}
}

#endif /* RWLIBS_CALIBRATION_DHLINKJACOBIAN_HPP_ */
