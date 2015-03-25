/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWLIBS_CALIBRATION_XMLCALIBRATIONMEASUREMENTFILE_HPP
#define RWLIBS_CALIBRATION_XMLCALIBRATIONMEASUREMENTFILE_HPP

#include <rw/common/DOMParser.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>

#include <rwlibs/calibration/CalibrationMeasurement.hpp>

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/


/**
 * @brief Interface specification for serialization of detection information
 */
class XMLDetectionInfoBaseSerializer {
public:
	/**
	 * @brief unserialize the detection info of \b element and store the data into \b measurement
	 */
	virtual void unserialize(CalibrationMeasurement::Ptr measurement, rw::common::DOMElem::Ptr element) {}

	/**
	 * @brief serialize the detection info of \b measurement and add to \b element
	 */
	virtual void serialize(CalibrationMeasurement::Ptr measurement, rw::common::DOMElem::Ptr element) {}
};


//Definition of string ID's
namespace {
	//Main ID for the collection of calibration measurements
	const std::string CALIBRATION_MEASUREMENTS_ID = "CalibrationMeasurements";
	//ID for a single measurement
	const std::string CALIBRATION_MEASUREMENT_ID = "CalibrationMeasurement";
	//ID for the device name
	const std::string DEVICE_NAME_ID = "DeviceName";	
	//ID for the marker frame
	const std::string MARKER_FRAME_ID = "MarkerFrame";
	//ID for the sensor frame
	const std::string SENSOR_FRAME_ID = "SensorFrame";
	//ID for the Covariance matrix
	const std::string COVARIANCE_MATRIX_ID = "CovarianceMatrix";
}

/**
 *@brief Handles serialization and unserialization of calibration measurements
 *
 * The template argument of the class is a XMLDetectionInfoSerializer which are used for serialize and unserialize
 * the detection information. 
 */
template <class INFOSERIALIZER = XMLDetectionInfoBaseSerializer>
class XMLCalibrationMeasurementFile {
public:
	/**
	 * @brief Loads \b filename and returns the measurements
	 * 
	 * Throw an exception in case of parse errors
	 */
	static std::vector<CalibrationMeasurement::Ptr> load(const std::string& filename);

	/**
	 * @brief Saves \b measurements to \b filename
	 */
	static void save(const std::vector<CalibrationMeasurement::Ptr>& measurements, const std::string& filename);
private:
	static CalibrationMeasurement::Ptr convertDomElementToMeasurement(rw::common::DOMElem::Ptr element);
	static void addMeasurementToDomElement(const CalibrationMeasurement::Ptr measurement, rw::common::DOMElem::Ptr parent);	
};


//Adds a measurement to a dom element
template <class INFOSERIALIZER>
void XMLCalibrationMeasurementFile<INFOSERIALIZER>::addMeasurementToDomElement(const CalibrationMeasurement::Ptr measurement, rw::common::DOMElem::Ptr parent) {
	using namespace rw::common;
	using namespace rw::loaders;
	DOMElem::Ptr elmMeasurement = parent->addChild(CALIBRATION_MEASUREMENT_ID);

	rw::math::Q q = measurement->getQ();
	DOMElem::Ptr elmState = DOMBasisTypes::write(q, elmMeasurement->addChild( DOMBasisTypes::QId ) );

	rw::math::Transform3D<> transform = measurement->getTransform();
	DOMElem::Ptr elmTransform = DOMBasisTypes::write(transform, elmMeasurement->addChild( DOMBasisTypes::Transform3DId ));

	
	std::string deviceName = measurement->getDeviceName();
	DOMBasisTypes::createElement(DEVICE_NAME_ID, deviceName, elmMeasurement);

	std::string markerFrameName = measurement->getMarkerFrameName();
	DOMBasisTypes::createElement(MARKER_FRAME_ID, markerFrameName, elmMeasurement);

	std::string sensorFrameName = measurement->getSensorFrameName();
	DOMBasisTypes::createElement(SENSOR_FRAME_ID, sensorFrameName, elmMeasurement);


	if (measurement->hasCovarianceMatrix()) {
		DOMElem::Ptr elmCovarianceMatrix = elmMeasurement->addChild(COVARIANCE_MATRIX_ID);
		Eigen::MatrixXd covarianceMatrix = measurement->getCovarianceMatrix();
		DOMBasisTypes::write( covarianceMatrix, elmCovarianceMatrix, false);
	}

	INFOSERIALIZER serializer;
	serializer.serialize(measurement, elmMeasurement);
}


//Implementation of the save method
template <class INFOSERIALIZER>
void XMLCalibrationMeasurementFile<INFOSERIALIZER>::save(const std::vector<CalibrationMeasurement::Ptr>& measurements, const std::string& filename) 
{
	using namespace rw::common;

	DOMParser::Ptr parser = DOMParser::make();

	DOMElem::Ptr elmRoot = parser->getRootElement();

	DOMElem::Ptr element = elmRoot->addChild(CALIBRATION_MEASUREMENTS_ID);

	for (std::vector<CalibrationMeasurement::Ptr>::const_iterator it = measurements.begin(); it != measurements.end(); ++it) {
		addMeasurementToDomElement(*it, element);
	}

	parser->save( filename );
}




//Unserializes a DOM element with a CalibrationMeasurement
template <class INFOSERIALIZER>
CalibrationMeasurement::Ptr XMLCalibrationMeasurementFile<INFOSERIALIZER>::convertDomElementToMeasurement(rw::common::DOMElem::Ptr element) 
{
	using namespace rw::common;
	using namespace rw::loaders;

	if ( !element->isName(CALIBRATION_MEASUREMENT_ID) && !element->isName("SerialDevicePoseMeasurement"))
		RW_THROW("Element not parsed correctly.");

	DOMElem::Ptr elmState = element->getChild( DOMBasisTypes::QId );
	rw::math::Q q = DOMBasisTypes::readQ(elmState, false);

	rw::math::Transform3D<> transform;
	if( element->hasChild(DOMBasisTypes::Transform3DId) ){
		DOMElem::Ptr elmTransform = element->getChild( DOMBasisTypes::Transform3DId );
		transform = DOMBasisTypes::readTransform3D(elmTransform, false);
	}
	if( element->hasChild(DOMBasisTypes::QuaternionId) ){
		DOMElem::Ptr elmTransform = element->getChild( DOMBasisTypes::QuaternionId );
		transform.R() = DOMBasisTypes::readQuaternion(elmTransform, false).toRotation3D();
	}
	if( element->hasChild(DOMBasisTypes::Vector3DId) ){
		DOMElem::Ptr elmTransform = element->getChild( DOMBasisTypes::Vector3DId );
		transform.P() = DOMBasisTypes::readVector3D(elmTransform, false);
	}


	CalibrationMeasurement::Ptr measurement = ownedPtr(new CalibrationMeasurement(q, transform));

	if (element->hasChild(DEVICE_NAME_ID)) {
		DOMElem::Ptr elmDeviceName = element->getChild(DEVICE_NAME_ID);
		std::string deviceName = DOMBasisTypes::readString(elmDeviceName, false);
		measurement->setDeviceName(deviceName);
	}

	if (element->hasChild(MARKER_FRAME_ID)) {
		DOMElem::Ptr elmMarkerFrame = element->getChild(MARKER_FRAME_ID);
		std::string markerFrame = DOMBasisTypes::readString(elmMarkerFrame, false);
		measurement->setMarkerFrameName(markerFrame);
	}
	
	if (element->hasChild(SENSOR_FRAME_ID)) {
		DOMElem::Ptr elmSensorFrame = element->getChild(SENSOR_FRAME_ID);
		std::string sensorFrame = DOMBasisTypes::readString(elmSensorFrame, false);
		measurement->setSensorFrameName(sensorFrame);
	}

	Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(6,6);
	if( element->hasChild(COVARIANCE_MATRIX_ID) ){
		DOMElem::Ptr elmMatrix = element->getChild(COVARIANCE_MATRIX_ID);
		covariance = DOMBasisTypes::readMatrix(elmMatrix);
		measurement->setCovarianceMatrix(covariance);
	} 

	
	INFOSERIALIZER serializer;
	serializer.unserialize(measurement, element);

	return measurement;
}


//Implementation of the load method
template <class INFOSERIALIZER>
std::vector<CalibrationMeasurement::Ptr> XMLCalibrationMeasurementFile<INFOSERIALIZER>::load(const std::string& filename) 
{
	using namespace rw::common;
	DOMParser::Ptr parser = DOMParser::make();

	parser->load(filename);

	DOMElem::Ptr elmRoot = parser->getRootElement();
	
	DOMElem::Ptr element;
	if( elmRoot->hasChild(CALIBRATION_MEASUREMENTS_ID)) {
		element = elmRoot->getChild(CALIBRATION_MEASUREMENTS_ID);
	} else if (elmRoot->hasChild("SerialDevicePoseMeasurements")) { //To maintain backwards capability we also check for the old SerialDevicePoseMeasurements ID
		element = elmRoot->getChild("SerialDevicePoseMeasurements");
	}
	else {
		RW_THROW("No CalibrationMeasurements tag found in measurement file... aborting");
	}

	std::vector<CalibrationMeasurement::Ptr> measurements;
	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren() ){
		CalibrationMeasurement::Ptr measurement = convertDomElementToMeasurement(child);
		measurements.push_back(measurement);
	}

	return measurements;
}


/* @} */

} //end namespace calibration
} //end namespace rwlibs



#endif //RWLIBS_CALIBRATION_XMLCALIBRATIONMEASUREMENTFILE_HPP
