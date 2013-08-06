/*
 * SerialDeviceCalibration.hpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP

#include "CompositeCalibration.hpp"
#include "DHLinkCalibration.hpp"
#include "JointEncoderCalibration.hpp"
#include "FixedFrameCalibration.hpp"

#include <rw/models.hpp>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

class SerialDeviceCalibration: public CompositeCalibration<Calibration> {
public:
	typedef rw::common::Ptr<SerialDeviceCalibration> Ptr;

	SerialDeviceCalibration(rw::models::SerialDevice::Ptr device,
			const std::vector<rw::math::Function<>::Ptr>& encoderCorrectionFunctions = std::vector<rw::math::Function<>::Ptr>());

	SerialDeviceCalibration(rw::models::SerialDevice::Ptr device,
			FixedFrameCalibration::Ptr baseCalibration,
			FixedFrameCalibration::Ptr endCalibration,
			CompositeCalibration<DHLinkCalibration>::Ptr compositeLinkCalibration,
			CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration);

	virtual ~SerialDeviceCalibration();

	rw::models::SerialDevice::Ptr getDevice() const;

	FixedFrameCalibration::Ptr getBaseCalibration() const;

	FixedFrameCalibration::Ptr getEndCalibration() const;

	CompositeCalibration<DHLinkCalibration>::Ptr getCompositeLinkCalibration() const;

	CompositeCalibration<JointEncoderCalibration>::Ptr getCompositeJointCalibration() const;

	static SerialDeviceCalibration::Ptr make(rw::models::SerialDevice::Ptr device);

	static SerialDeviceCalibration::Ptr get(rw::models::SerialDevice::Ptr device);

	static SerialDeviceCalibration::Ptr get(const rw::common::PropertyMap& propertyMap);

	static void set(SerialDeviceCalibration::Ptr calibration, rw::models::SerialDevice::Ptr device);

	static void set(SerialDeviceCalibration::Ptr calibration, rw::common::PropertyMap& propertyMap);

private:
	rw::models::SerialDevice::Ptr _device;
	FixedFrameCalibration::Ptr _baseCalibration;
	FixedFrameCalibration::Ptr _endCalibration;
	CompositeCalibration<DHLinkCalibration>::Ptr _compositeLinkCalibration;
	CompositeCalibration<JointEncoderCalibration>::Ptr _compositeJointCalibration;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICECALIBRATION_HPP */
