/*
 * SerialDeviceJacobian.hpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICEJACOBIAN_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICEJACOBIAN_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "CompositeJacobian.hpp"
#include "DHLinkJacobian.hpp"
#include "FixedFrameJacobian.hpp"
#include "JointEncoderJacobian.hpp"
#include "SerialDeviceCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

class SerialDeviceJacobian: public CompositeJacobian<Jacobian> {
public:
	typedef rw::common::Ptr<SerialDeviceJacobian> Ptr;

	SerialDeviceJacobian(SerialDeviceCalibration::Ptr calibration);

	virtual ~SerialDeviceJacobian();

	FixedFrameJacobian::Ptr getBaseJacobian() const;

	FixedFrameJacobian::Ptr getEndJacobian() const;

	CompositeJacobian<DHLinkJacobian>::Ptr getCompositeLinkJacobian() const;

	CompositeJacobian<JointEncoderJacobian>::Ptr getCompositeJointJacobian() const;

private:
	FixedFrameJacobian::Ptr _baseJacobian;
	FixedFrameJacobian::Ptr _endJacobian;
	CompositeJacobian<DHLinkJacobian>::Ptr _compositeLinkJacobian;
	CompositeJacobian<JointEncoderJacobian>::Ptr _compositeJointJacobian;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICEJACOBIAN_HPP */
