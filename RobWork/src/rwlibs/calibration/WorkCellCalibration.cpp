/*
 * WorkCellCalibration.cpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#include "WorkCellCalibration.hpp"

#include <rw/models.hpp>

#include <boost/foreach.hpp>



using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rwlibs::calibration;


WorkCellCalibration::WorkCellCalibration(std::vector<std::pair<SerialDevice::Ptr, Frame*> > deviceMarkerPairs,
										 const std::vector<Frame*>& sensorFrames,
										 const std::vector<Function<>::Ptr>& encoderCorrectionFunctions) 
										 :	_deviceMarkerPairs(deviceMarkerPairs)
{

	//The calibration of the moving frame. Typically the camera frame, but could also be other frames.
	_fixedFrameCalibrations = ownedPtr(new CompositeCalibration<FixedFrameCalibration>());
	BOOST_FOREACH(Frame* sensorFrame, sensorFrames) {
		if (dynamic_cast<FixedFrame*>(sensorFrame) == NULL) {
			RW_THROW("The frame '"<<sensorFrame->getName()<<"' is not a FixedFrame as required by the calibration.");
		}

		FixedFrameCalibration::Ptr sensorFrameCalibration = rw::common::ownedPtr(new FixedFrameCalibration(rw::kinematics::Frame::Ptr(sensorFrame).cast<rw::kinematics::FixedFrame>()));
		_fixedFrameCalibrations->addCalibration(sensorFrameCalibration);
		_sensorFrameCalibrations[sensorFrame->getName()] = sensorFrameCalibration;
	}

	//_movingFrameCalibration = rw::common::ownedPtr(new FixedFrameCalibration(movingFrame, true));

	
	BOOST_FOREACH(DeviceMarkerPair deviceMarkerPair, deviceMarkerPairs) {
		FixedFrameCalibration::Ptr endCalibration = rw::common::ownedPtr(new FixedFrameCalibration(rw::kinematics::Frame::Ptr(deviceMarkerPair.second).cast<rw::kinematics::FixedFrame>()));
		_markerCalibrations[deviceMarkerPair.first->getName()] = endCalibration;
		_fixedFrameCalibrations->addCalibration(endCalibration);
		
	//_endCalibration = rw::common::ownedPtr(new FixedFrameCalibration(device->getEnd(), false));
	
		_compositeLinkCalibration = rw::common::ownedPtr(new CompositeCalibration<ParallelAxisDHCalibration>());
		_compositeJointCalibration = rw::common::ownedPtr(new CompositeCalibration<JointEncoderCalibration>());

		std::vector<rw::models::Joint*> joints = deviceMarkerPair.first->getJoints();
		for (std::vector<rw::models::Joint*>::iterator jointIterator = joints.begin(); jointIterator != joints.end(); jointIterator++) {
			rw::models::Joint::Ptr joint = (*jointIterator);

			// Add link calibrations for intermediate links.
			if (jointIterator != joints.begin() ) {
				ParallelAxisDHCalibration::Ptr linkCalibration = rw::common::ownedPtr(new ParallelAxisDHCalibration(joint));
				_compositeLinkCalibration->addCalibration(linkCalibration);

				// Disable d and theta for first link.
				if (jointIterator == ++(joints.begin())) {
					CalibrationParameterSet parameterSet = linkCalibration->getParameterSet();
					parameterSet(ParallelAxisDHCalibration::PARAMETER_D).setEnabled(false);
					parameterSet(ParallelAxisDHCalibration::PARAMETER_THETA).setEnabled(false);
					parameterSet(ParallelAxisDHCalibration::PARAMETER_A).setEnabled(true);
					parameterSet(ParallelAxisDHCalibration::PARAMETER_ALPHA).setEnabled(true);
					parameterSet(ParallelAxisDHCalibration::PARAMETER_B).setEnabled(false);
					parameterSet(ParallelAxisDHCalibration::PARAMETER_BETA).setEnabled(false);
				
					linkCalibration->setParameterSet(parameterSet);
					//break;
				}
			}

			// Add joint calibrations.
			JointEncoderCalibration::Ptr jointCalibration = rw::common::ownedPtr(new JointEncoderCalibration(deviceMarkerPair.first.cast<rw::models::JointDevice>(), joint));
			_compositeJointCalibration->addCalibration(jointCalibration);
		}
	}

	((CompositeCalibration<Calibration>*) this)->addCalibration(_fixedFrameCalibrations.cast<Calibration>());
	((CompositeCalibration<Calibration>*) this)->addCalibration(_compositeLinkCalibration.cast<Calibration>());
	((CompositeCalibration<Calibration>*) this)->addCalibration(_compositeJointCalibration.cast<Calibration>());
}

		
WorkCellCalibration::WorkCellCalibration(/*rw::models::SerialDevice::Ptr device,*/
		/*FixedFrameCalibration::Ptr sensorFrameCalibration,
		FixedFrameCalibration::Ptr endCalibration,*/
		CompositeCalibration<FixedFrameCalibration>::Ptr fixedFrameCalibrations,
		CompositeCalibration<ParallelAxisDHCalibration>::Ptr compositeLinkCalibration,
		CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration) :
			//_device(device),
			//_sensorFrameCalibration(sensorFrameCalibration), 
			//_endCalibration(endCalibration),
			_fixedFrameCalibrations(fixedFrameCalibrations),
			_compositeLinkCalibration(compositeLinkCalibration),
			_compositeJointCalibration(compositeJointCalibration)
{
	//_fixedFrameCalibrations = ownedPtr(new CompositeCalibration<FixedFrameCalibration>());
	//_fixedFrameCalibrations->addCalibration(sensorFrameCalibration);
	//_fixedFrameCalibrations->addCalibration(endCalibration);
	((CompositeCalibration<Calibration>*) this)->addCalibration(_fixedFrameCalibrations.cast<Calibration>());
	//((CompositeCalibration<Calibration>*) this)->addCalibration(_sensorFrameCalibration.cast<Calibration>());
	//((CompositeCalibration<Calibration>*) this)->addCalibration(_endCalibration.cast<Calibration>());
	((CompositeCalibration<Calibration>*) this)->addCalibration(_compositeLinkCalibration.cast<Calibration>());
	((CompositeCalibration<Calibration>*) this)->addCalibration(_compositeJointCalibration.cast<Calibration>());
}

WorkCellCalibration::~WorkCellCalibration() {

}

FixedFrameCalibration::Ptr WorkCellCalibration::getFixedFrameCalibrationForSensor(const std::string& sensor) {
	if (_sensorFrameCalibrations.find(sensor) != _sensorFrameCalibrations.end()) {
		return _sensorFrameCalibrations[sensor];
	}
	return NULL;
}

FixedFrameCalibration::Ptr WorkCellCalibration::getFixedFrameCalibrationForMarker(const std::string& marker) {
	if (_markerCalibrations.find(marker) != _markerCalibrations.end()) {
		return _markerCalibrations[marker];
	}
	return NULL;

}


/*
rw::models::SerialDevice::Ptr WorkCellCalibration::getDevice() const {
	return _device;
}*/

/*
FixedFrameCalibration::Ptr WorkCellCalibration::getBaseCalibration() const {
	return _sensorFrameCalibration;
}

FixedFrameCalibration::Ptr WorkCellCalibration::getEndCalibration() const {
	return _endCalibration;
}
*/

CompositeCalibration<FixedFrameCalibration>::Ptr WorkCellCalibration::getFixedFrameCalibrations() const {
	return _fixedFrameCalibrations;
}

CompositeCalibration<ParallelAxisDHCalibration>::Ptr WorkCellCalibration::getCompositeLinkCalibration() const {
	return _compositeLinkCalibration;
}
 
CompositeCalibration<JointEncoderCalibration>::Ptr WorkCellCalibration::getCompositeJointCalibration() const {
	return _compositeJointCalibration;
}

WorkCellCalibration::Ptr WorkCellCalibration::get(rw::models::SerialDevice::Ptr device) {
	return get(device->getPropertyMap());
}

WorkCellCalibration::Ptr WorkCellCalibration::get(const rw::common::PropertyMap& propertyMap) {
	WorkCellCalibration::Ptr calibration;
	if (propertyMap.has("Calibration"))
		calibration = propertyMap.get<WorkCellCalibration::Ptr>("Calibration");
	return calibration;
}

void WorkCellCalibration::set(WorkCellCalibration::Ptr calibration, rw::models::SerialDevice::Ptr device) {
	set(calibration, device->getPropertyMap());
}

void WorkCellCalibration::set(WorkCellCalibration::Ptr calibration, rw::common::PropertyMap& propertyMap) {
	if (propertyMap.has("Calibration")) {
		if (calibration->isApplied())
			calibration->revert();
		propertyMap.erase("Calibration");
	}
	propertyMap.add<WorkCellCalibration::Ptr>("Calibration", "Calibration of SerialDevice", calibration);
}


void WorkCellCalibration::prependCalibration(WorkCellCalibration::Ptr calibration) 
{
	typedef std::map<std::string, FixedFrameCalibration::Ptr> StringCalibMap;
	for (StringCalibMap::iterator it = _markerCalibrations.begin(); it != _markerCalibrations.end(); ++it) {
		FixedFrameCalibration::Ptr ffc = calibration->getFixedFrameCalibrationForMarker((*it).first);
		Transform3D<> oldTransform = (*it).second->getCorrectionTransform();
		Transform3D<> preTransform = ffc->getCorrectionTransform();
		Transform3D<> newTransform = preTransform*oldTransform;
		(*it).second->setCorrectionTransform(newTransform);
	}
	

	for (StringCalibMap::iterator it = _sensorFrameCalibrations.begin(); it != _sensorFrameCalibrations.end(); ++it) {
		FixedFrameCalibration::Ptr ffc = calibration->getFixedFrameCalibrationForSensor((*it).first);
		Transform3D<> oldTransform = (*it).second->getCorrectionTransform();
		Transform3D<> preTransform = ffc->getCorrectionTransform();
		Transform3D<> newTransform = preTransform*oldTransform;
		(*it).second->setCorrectionTransform(newTransform);
	}

}

