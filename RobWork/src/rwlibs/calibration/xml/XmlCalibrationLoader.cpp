/*
 * XmlCalibrationFile.cpp
 *
 *  Created on: Sep 18, 2012
 *      Author: bing
 */

#include "XmlCalibrationLoader.hpp"

#include <QtCore>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class ElementReader {
public:
	ElementReader(rw::kinematics::StateStructure::Ptr stateStructure, rw::models::SerialDevice::Ptr serialDevice) :
			_stateStructure(stateStructure), _serialDevice(serialDevice) {

	}

	template<class T>
	T readElement(const QDomElement& element);

private:
	rw::kinematics::StateStructure::Ptr _stateStructure;
	rw::models::SerialDevice::Ptr _serialDevice;
};

template<>
FixedFrameCalibration::Ptr ElementReader::readElement<FixedFrameCalibration::Ptr>(const QDomElement& element) {
	if (!element.hasAttribute("frame"))
		RW_THROW("\"frame\" attribute missing.");
	std::string frameName = element.attribute("frame").toStdString();
	rw::kinematics::Frame* frame = _stateStructure->findFrame(frameName);
	rw::kinematics::FixedFrame::Ptr fixedFrame = rw::kinematics::Frame::Ptr(frame).cast<rw::kinematics::FixedFrame>();
	if (fixedFrame.isNull())
		RW_THROW("Frame \"" << frameName << "\" not found.");
	
	QDomElement transformElement = element.namedItem("Transform").toElement();
	if (transformElement.isNull())
		RW_THROW("\"Transform\" element not found");

	if (!transformElement.hasAttribute("isPostCorrection"))
		RW_THROW("\"isPostCorrection\" attribute missing.");
	bool isPostCorrection = transformElement.attribute("isPostCorrection").toInt();

	QStringList txtTransformSplitted = transformElement.text().simplified().split(" ");
	txtTransformSplitted.removeAll(" ");
	if (txtTransformSplitted.count() != 12)
		RW_THROW("Transform has wrong size (12 numbers).");
	rw::math::Transform3D<> transform;
	for (int rowIndex = 0; rowIndex < 3; rowIndex++)
		for (int columnIndex = 0; columnIndex < 4; columnIndex++)
			transform(rowIndex, columnIndex) = txtTransformSplitted[4 * rowIndex + columnIndex].toDouble();

	return rw::common::ownedPtr(new FixedFrameCalibration(fixedFrame, isPostCorrection, transform));
}

template<>
DHLinkCalibration::Ptr ElementReader::readElement<DHLinkCalibration::Ptr>(const QDomElement& element) {
	if (!element.hasAttribute("joint"))
		RW_THROW("\"joint\" attribute missing.");
	std::string jointName = element.attribute("joint").toStdString();

	rw::models::Joint::Ptr joint = (rw::models::Joint*) _stateStructure->findFrame(jointName);
	if (joint.isNull())
		RW_THROW("Joint \"" << jointName << "\" not found.");

	DHLinkCalibration::Ptr calibration = rw::common::ownedPtr(new DHLinkCalibration(joint));
	CalibrationParameterSet parameterSet = calibration->getParameterSet();

	if (!element.hasAttribute("a"))
		parameterSet(DHLinkCalibration::PARAMETER_A).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_A) = element.attribute("a").toDouble();

	if (!element.hasAttribute("b"))
		parameterSet(DHLinkCalibration::PARAMETER_B).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_B) = element.attribute("b").toDouble();

	if (!element.hasAttribute("d"))
		parameterSet(DHLinkCalibration::PARAMETER_D).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_D) = element.attribute("d").toDouble();

	if (!element.hasAttribute("alpha"))
		parameterSet(DHLinkCalibration::PARAMETER_ALPHA).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_ALPHA) = element.attribute("alpha").toDouble();

	if (!element.hasAttribute("beta"))
		parameterSet(DHLinkCalibration::PARAMETER_BETA).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_BETA) = element.attribute("beta").toDouble();

	if (!element.hasAttribute("theta"))
		parameterSet(DHLinkCalibration::PARAMETER_THETA).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_THETA) = element.attribute("theta").toDouble();

	calibration->setParameterSet(parameterSet);

	return calibration;
}

SerialDeviceCalibration::Ptr XmlCalibrationLoader::load(rw::kinematics::StateStructure::Ptr stateStructure,
		rw::models::SerialDevice::Ptr device, std::string fileName) {
	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::ReadOnly | QIODevice::Text | QIODevice::Truncate);

	QDomDocument document("SerialDeviceCalibration");
	if (!document.setContent(&file))
		RW_THROW("Content not set.");

	QDomElement elmRoot = document.documentElement();
	if (elmRoot.tagName() != "SerialDeviceCalibration")
		RW_THROW("Element not found.");

	ElementReader elementReader(stateStructure, device);

	// Load base calibration.
	FixedFrameCalibration::Ptr baseCalibration;
	QDomNode nodeBase = elmRoot.namedItem("BaseCalibration");
	if (!nodeBase.isNull() && nodeBase.hasChildNodes())
		baseCalibration = elementReader.readElement<FixedFrameCalibration::Ptr>(nodeBase.childNodes().at(0).toElement());

	// Load end calibration.
	FixedFrameCalibration::Ptr endCalibration;
	QDomNode nodeEnd = elmRoot.namedItem("EndCalibration");
	if (!nodeEnd.isNull() && nodeEnd.hasChildNodes())
		endCalibration = elementReader.readElement<FixedFrameCalibration::Ptr>(nodeEnd.childNodes().at(0).toElement());

	// Load link calibrations.
	CompositeCalibration<DHLinkCalibration>::Ptr compositeLinkCalibration = rw::common::ownedPtr(new CompositeCalibration<DHLinkCalibration>());
	QDomNode nodeLinks = elmRoot.namedItem("LinkCalibrations");
	if (!nodeLinks.isNull()) {
		QDomNodeList nodes = nodeLinks.childNodes();
		for (int nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++) {
			DHLinkCalibration::Ptr linkCalibration = elementReader.readElement<DHLinkCalibration::Ptr>(nodes.at(nodeIndex).toElement());
			compositeLinkCalibration->addCalibration(linkCalibration);
		}
	}

	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = rw::common::ownedPtr(new CompositeCalibration<JointEncoderCalibration>());

	SerialDeviceCalibration::Ptr calibration = rw::common::ownedPtr(new SerialDeviceCalibration(device, baseCalibration, endCalibration, compositeLinkCalibration, compositeJointCalibration));

	return calibration;
}

}
}
