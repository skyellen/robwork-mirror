/*
 * DHLinkJacobian.hpp
 */

#ifndef RWLIBS_CALIBRATION_PARALLELAXISDHJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_PARALLELAXISDHJACOBIAN_HPP_

#include <rw/math.hpp>
//#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "ParallelAxisDHCalibration.hpp"
#include "JacobianBase.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/models/DHParameterSet.hpp>

namespace rwlibs {
namespace calibration {


class ParallelAxisDHJacobian: public JacobianBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<ParallelAxisDHJacobian> Ptr;

	ParallelAxisDHJacobian(ParallelAxisDHCalibration::Ptr calibration);

	virtual ~ParallelAxisDHJacobian();

protected:
	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame, const rw::kinematics::State& state);

private:
	ParallelAxisDHCalibration::Ptr _calibration;
	rw::models::Joint::Ptr _joint;
};

}
}

#endif /* RWLIBS_CALIBRATION_PARALLELAXISDHJACOBIAN_HPP_ */
