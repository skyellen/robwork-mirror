///********************************************************************************
// * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
// * Faculty of Engineering, University of Southern Denmark
// *
// * Licensed under the Apache License, Version 2.0 (the "License");
// * you may not use this file except in compliance with the License.
// * You may obtain a copy of the License at
// *
// *     http://www.apache.org/licenses/LICENSE-2.0
// *
// * Unless required by applicable law or agreed to in writing, software
// * distributed under the License is distributed on an "AS IS" BASIS,
// * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// * See the License for the specific language governing permissions and
// * limitations under the License.
// ********************************************************************************/
//
//#include "XmlMeasurementFile.hpp"
//
//#include <rw/loaders/dom/DOMBasisTypes.hpp>
//
//#include <rw/common.hpp>
//#include <rw/common/DOMParser.hpp>
//#include <rw/common/DOMElem.hpp>
//
//using namespace rwlibs::calibration;
//using namespace rw::common;
//using namespace rw::loaders;
//using namespace rw;
//
//namespace {
//
//	/**
//	 * adds measurement xml description as child to parent node
//	 */
//	void addMeasurementToDomElement(const SerialDevicePoseMeasurement& measurement, DOMElem::Ptr parent) {
//		DOMElem::Ptr elmMeasurement = parent->addChild("SerialDevicePoseMeasurement");
//
//		rw::math::Q q = measurement.getQ();
//		DOMElem::Ptr elmState = DOMBasisTypes::write(q, elmMeasurement->addChild( DOMBasisTypes::QId ) );
//
//		rw::math::Transform3D<> transform = measurement.getTransform();
//		DOMElem::Ptr elmTransform = DOMBasisTypes::write(transform, elmMeasurement->addChild( DOMBasisTypes::Transform3DId ));
//
//		if (measurement.hasCovarianceMatrix()) {
//			DOMElem::Ptr elmCovarianceMatrix = elmMeasurement->addChild("CovarianceMatrix");
//			Eigen::Matrix<double, 6, 6> covarianceMatrix = measurement.getCovarianceMatrix();
//			DOMBasisTypes::write( covarianceMatrix, elmCovarianceMatrix, false);
//		}
//	}
//
//	/// get mesurement from dom node
//	SerialDevicePoseMeasurement convertDomElementToMeasurement(DOMElem::Ptr element) {
//		if ( !element->isName("SerialDevicePoseMeasurement") )
//			RW_THROW("Element not parsed correctly.");
//
//		DOMElem::Ptr elmState = element->getChild( DOMBasisTypes::QId );
//		rw::math::Q q = DOMBasisTypes::readQ(elmState, false);
//
//		DOMElem::Ptr elmTransform = element->getChild( DOMBasisTypes::Transform3DId );
//		rw::math::Transform3D<> transform = DOMBasisTypes::readTransform3D(elmTransform, false);
//
//		Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
//		if( element->hasChild("CovarianceMatrix") ){
//			DOMElem::Ptr elmMatrix = element->getChild("CovarianceMatrix");
//			covariance = DOMBasisTypes::readMatrix(elmMatrix);
//		} 
//
//		return SerialDevicePoseMeasurement(q, transform, covariance);
//	}
//}
//void XmlMeasurementFile::save(const std::vector<SerialDevicePoseMeasurement>& measurements, std::string fileName) {
//	DOMParser::Ptr parser = DOMParser::make();
//
//	DOMElem::Ptr elmRoot = parser->getRootElement();
//
//	DOMElem::Ptr element = elmRoot->addChild("SerialDevicePoseMeasurements");
//
//	for (std::vector<SerialDevicePoseMeasurement>::const_iterator it = measurements.begin(); it != measurements.end(); ++it) {
//		const SerialDevicePoseMeasurement measurement = (*it);
//		addMeasurementToDomElement(measurement, element);
//	}
//
//	parser->save( fileName );
//}
//
//std::vector<SerialDevicePoseMeasurement> XmlMeasurementFile::load(std::string fileName) {
//	DOMParser::Ptr parser = DOMParser::make();
//
//	parser->load(fileName);
//
//	DOMElem::Ptr elmRoot = parser->getRootElement();
//
//	if( !elmRoot->hasChild("SerialDevicePoseMeasurements") ){
//		RW_THROW("No SerialDevicePoseMeasurements tag found in measurement file... aborting");
//	}
//
//	DOMElem::Ptr element = elmRoot->getChild("SerialDevicePoseMeasurements");
//
//	std::vector<SerialDevicePoseMeasurement> measurements;
//	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren() ){
//		SerialDevicePoseMeasurement measurement = convertDomElementToMeasurement(child);
//		measurements.push_back(measurement);
//	}
//
//	return measurements;
//}
