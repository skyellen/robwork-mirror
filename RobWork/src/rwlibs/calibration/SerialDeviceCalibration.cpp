/*
 * SerialDeviceCalibration.cpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#include "SerialDeviceCalibration.hpp"

#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

using namespace rwlibs::calibration;

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr device,
		const std::vector<rw::math::Function<>::Ptr>& encoderCorrectionFunctions) :
		_device(device)
{
	_baseCalibration = rw::common::ownedPtr(
			new FixedFrameCalibration(rw::kinematics::Frame::Ptr(device->getBase()).cast<rw::kinematics::FixedFrame>(), true));

	_endCalibration = rw::common::ownedPtr(
		new FixedFrameCalibration(rw::kinematics::Frame::Ptr(device->getEnd()).cast<rw::kinematics::FixedFrame>(), false));
	
	_compositeLinkCalibration = rw::common::ownedPtr(new CompositeCalibration<DHLinkCalibration>());
	_compositeJointCalibration = rw::common::ownedPtr(new CompositeCalibration<JointEncoderCalibration>());

	std::vector<rw::models::Joint*> joints = device->getJoints();
	for (std::vector<rw::models::Joint*>::iterator jointIterator = joints.begin(); jointIterator != joints.end(); jointIterator++) {
		rw::models::Joint::Ptr joint = (*jointIterator);

		// Add link calibrations for intermediate links.
		if (jointIterator != joints.begin()) {
			DHLinkCalibration::Ptr linkCalibration = rw::common::ownedPtr(new DHLinkCalibration(joint));
			_compositeLinkCalibration->addCalibration(linkCalibration);

			// Disable d and theta for first link.
			if (jointIterator == ++(joints.begin())) {
				CalibrationParameterSet parameterSet = linkCalibration->getParameterSet();
				parameterSet(DHLinkCalibration::PARAMETER_D).setEnabled(false);
				parameterSet(DHLinkCalibration::PARAMETER_THETA).setEnabled(false);
				linkCalibration->setParameterSet(parameterSet);
			}
		}

		// Add joint calibrations.
		JointEncoderCalibration::Ptr jointCalibration = rw::common::ownedPtr(new JointEncoderCalibration(device.cast<rw::models::JointDevice>(), joint));
		_compositeJointCalibration->addCalibration(jointCalibration);
	}
	
	((CompositeCalibration<Calibration>*) this)->addCalibration(_baseCalibration.cast<Calibration>());
	((CompositeCalibration<Calibration>*) this)->addCalibration(_endCalibration.cast<Calibration>());
	CompositeCalibration<Calibration>::addCalibration(_compositeLinkCalibration.cast<Calibration>());
	CompositeCalibration<Calibration>::addCalibration(_compositeJointCalibration.cast<Calibration>());
}

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr device,
		FixedFrameCalibration::Ptr baseCalibration,
		FixedFrameCalibration::Ptr endCalibration,
		CompositeCalibration<DHLinkCalibration>::Ptr compositeLinkCalibration,
		CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration) :
			_device(device),
			_baseCalibration(baseCalibration),
			_endCalibration(endCalibration),
			_compositeLinkCalibration(compositeLinkCalibration),
			_compositeJointCalibration(compositeJointCalibration)
{
	CompositeCalibration<Calibration>::addCalibration(_baseCalibration.cast<Calibration>());
	CompositeCalibration<Calibration>::addCalibration(_endCalibration.cast<Calibration>());
	CompositeCalibration<Calibration>::addCalibration(_compositeLinkCalibration.cast<Calibration>());
	CompositeCalibration<Calibration>::addCalibration(_compositeJointCalibration.cast<Calibration>());
}

SerialDeviceCalibration::~SerialDeviceCalibration() {

}

rw::models::SerialDevice::Ptr SerialDeviceCalibration::getDevice() const {
	return _device;
}

FixedFrameCalibration::Ptr SerialDeviceCalibration::getBaseCalibration() const {
	return _baseCalibration;
}

FixedFrameCalibration::Ptr SerialDeviceCalibration::getEndCalibration() const {
	return _endCalibration;
}

CompositeCalibration<DHLinkCalibration>::Ptr SerialDeviceCalibration::getCompositeLinkCalibration() const {
	return _compositeLinkCalibration;
}

CompositeCalibration<JointEncoderCalibration>::Ptr SerialDeviceCalibration::getCompositeJointCalibration() const {
	return _compositeJointCalibration;
}

SerialDeviceCalibration::Ptr SerialDeviceCalibration::get(rw::models::SerialDevice::Ptr device) {
	return get(device->getPropertyMap());
}

SerialDeviceCalibration::Ptr SerialDeviceCalibration::get(const rw::common::PropertyMap& propertyMap) {
	SerialDeviceCalibration::Ptr calibration;
	if (propertyMap.has("Calibration"))
		calibration = propertyMap.get<SerialDeviceCalibration::Ptr>("Calibration");
	return calibration;
}

void SerialDeviceCalibration::set(SerialDeviceCalibration::Ptr calibration, rw::models::SerialDevice::Ptr device) {
	set(calibration, device->getPropertyMap());
}

void SerialDeviceCalibration::set(SerialDeviceCalibration::Ptr calibration, rw::common::PropertyMap& propertyMap) {
	if (propertyMap.has("Calibration")) {
		if (calibration->isApplied())
			calibration->revert();
		propertyMap.erase("Calibration");
	}
	propertyMap.add<SerialDeviceCalibration::Ptr>("Calibration", "Calibration of SerialDevice", calibration);
}

}
}
