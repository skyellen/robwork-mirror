/*
 * WorkCellJacobian.hpp
 */

#ifndef RWLIBS_CALIBRATION_WORKCELLJACOBIAN_HPP
#define RWLIBS_CALIBRATION_WORKCELLJACOBIAN_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "CompositeJacobian.hpp"
#include "ParallelAxisDHJacobian.hpp"
#include "FixedFrameJacobian.hpp"
#include "JointEncoderJacobian.hpp"
#include "WorkCellCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

class WorkCellJacobian: public CompositeJacobian<Jacobian> {
public:
	typedef rw::common::Ptr<WorkCellJacobian> Ptr;

	WorkCellJacobian(WorkCellCalibration::Ptr calibration);

	virtual ~WorkCellJacobian();

	FixedFrameJacobian::Ptr getBaseJacobian() const;

	FixedFrameJacobian::Ptr getEndJacobian() const;

	CompositeJacobian<ParallelAxisDHJacobian>::Ptr getCompositeLinkJacobian() const;

	CompositeJacobian<JointEncoderJacobian>::Ptr getCompositeJointJacobian() const;

private:
	FixedFrameJacobian::Ptr _baseJacobian;
	FixedFrameJacobian::Ptr _endJacobian;
	CompositeJacobian<FixedFrameJacobian>::Ptr _compositeFixedFrameJacobian;
	CompositeJacobian<ParallelAxisDHJacobian>::Ptr _compositeLinkJacobian;
	CompositeJacobian<JointEncoderJacobian>::Ptr _compositeJointJacobian;
};

}
}

#endif /* RWLIBS_CALIBRATION_WORKCELLJACOBIAN_HPP*/
