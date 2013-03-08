/*
 * XmlCalibrationFile.cpp
 *
 *  Created on: Sep 18, 2012
 *      Author: bing
 */

#include "XmlCalibrationLoader.hpp"

#include <rw/common/DOMElem.hpp>
#include <rw/common/DOMParser.hpp>

using namespace rwlibs::calibration;
using namespace rw::common;
using namespace rw;

class ElementReader {
public:
	ElementReader(rw::kinematics::StateStructure::Ptr stateStructure, rw::models::SerialDevice::Ptr serialDevice) :
			_stateStructure(stateStructure), _serialDevice(serialDevice) {

	}

	template<class T>
	T readElement(const DOMElem& element);

private:
	rw::kinematics::StateStructure::Ptr _stateStructure;
	rw::models::SerialDevice::Ptr _serialDevice;
};

template<>
FixedFrameCalibration::Ptr ElementReader::readElement<FixedFrameCalibration::Ptr>(const DOMElem& element) {
	if (!element->hasAttribute("frame"))
		RW_THROW("\"frame\" attribute missing.");
	std::string frameName = element->getAttributeValue("frame");
	rw::kinematics::Frame* frame = _stateStructure->findFrame(frameName);
	rw::kinematics::FixedFrame::Ptr fixedFrame = rw::kinematics::Frame::Ptr(frame).cast<rw::kinematics::FixedFrame>();
	if (fixedFrame.isNull())
		RW_THROW("Frame \"" << frameName << "\" not found.");
	
	DOMElem::Ptr transformElement = element->getChild("Transform");
	if (transformElement == NULL)
		RW_THROW("\"Transform\" element not found");

	Transform3D<> t3d = BasisTypes::readTransform3D(transformElement, false);

	if (!transformElement->hasAttribute("isPostCorrection"))
		RW_THROW("\"isPostCorrection\" attribute missing.");
	bool isPostCorrection = transformElement->getAttributeValueAsBool("isPostCorrection");

	return rw::common::ownedPtr(new FixedFrameCalibration(fixedFrame, isPostCorrection, transform));
}

template<>
DHLinkCalibration::Ptr ElementReader::readElement<DHLinkCalibration::Ptr>(const DOMElem& element) {
	if (!element->hasAttribute("joint"))
		RW_THROW("\"joint\" attribute missing.");
	std::string jointName = element->attribute("joint").toStdString();

	rw::models::Joint::Ptr joint = (rw::models::Joint*) _stateStructure->findFrame(jointName);
	if (joint.isNull())
		RW_THROW("Joint \"" << jointName << "\" not found.");

	DHLinkCalibration::Ptr calibration = rw::common::ownedPtr(new DHLinkCalibration(joint));
	CalibrationParameterSet parameterSet = calibration->getParameterSet();

	if (!element->hasAttribute("a"))
		parameterSet(DHLinkCalibration::PARAMETER_A).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_A) = element->attribute("a").toDouble();

	if (!element->hasAttribute("b"))
		parameterSet(DHLinkCalibration::PARAMETER_B).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_B) = element->attribute("b").toDouble();

	if (!element->hasAttribute("d"))
		parameterSet(DHLinkCalibration::PARAMETER_D).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_D) = element->attribute("d").toDouble();

	if (!element->hasAttribute("alpha"))
		parameterSet(DHLinkCalibration::PARAMETER_ALPHA).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_ALPHA) = element->attribute("alpha").toDouble();

	if (!element->hasAttribute("beta"))
		parameterSet(DHLinkCalibration::PARAMETER_BETA).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_BETA) = element->attribute("beta").toDouble();

	if (!element->hasAttribute("theta"))
		parameterSet(DHLinkCalibration::PARAMETER_THETA).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_THETA) = element->getAttributeValueAsDouble("theta");

	calibration->setParameterSet(parameterSet);

	return calibration;
}

SerialDeviceCalibration::Ptr XmlCalibrationLoader::load(
		rw::kinematics::StateStructure::Ptr stateStructure,
		rw::models::SerialDevice::Ptr device,
		std::string fileName)
{
	DOMParser::Ptr parser = DOMParser::make();

	parser->load(fileName);

	DOMElem::Ptr elmRoot = parser->getRootElement();
	if ( !elmRoot->isName("SerialDeviceCalibration") )
		RW_THROW("Element not found.");

	ElementReader elementReader(stateStructure, device);

	// Load base calibration.
	FixedFrameCalibration::Ptr baseCalibration;
	DOMElem::Ptr nodeBase = elmRoot->getChild("BaseCalibration");
	if (nodeBase!=NULL && nodeBase->hasChildren())
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
