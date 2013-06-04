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

#include "XmlCalibrationLoader.hpp"

#include <rw/common/DOMElem.hpp>
#include <rw/common/DOMParser.hpp>

#include <rw/loaders/dom/DOMBasisTypes.hpp>

using namespace rwlibs::calibration;
using namespace rw::common;
using namespace rw::math;
using namespace rw::loaders;
using namespace rw;

class ElementReader {
public:
	ElementReader(rw::kinematics::StateStructure::Ptr stateStructure, rw::models::SerialDevice::Ptr serialDevice) :
			_stateStructure(stateStructure), _serialDevice(serialDevice) {

	}

	template<class T>
	T readElement(DOMElem::Ptr element);

private:
	rw::kinematics::StateStructure::Ptr _stateStructure;
	rw::models::SerialDevice::Ptr _serialDevice;
};

template<>
FixedFrameCalibration::Ptr ElementReader::readElement<FixedFrameCalibration::Ptr>(DOMElem::Ptr felement) {
	DOMElem::Ptr element = felement->getChild("FixedFrameCalibration");

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

	Transform3D<> t3d = DOMBasisTypes::readTransform3D(transformElement, false);

	if (!transformElement->hasAttribute("isPostCorrection"))
		RW_THROW("\"isPostCorrection\" attribute missing.");
	bool isPostCorrection = transformElement->getAttributeValueAsBool("isPostCorrection");

	return rw::common::ownedPtr(new FixedFrameCalibration(fixedFrame, isPostCorrection, t3d));
}

template<>
DHLinkCalibration::Ptr ElementReader::readElement<DHLinkCalibration::Ptr>(DOMElem::Ptr element) {
	//DOMElem::Ptr element = lelement->getChild("DHLinkCalibration");

	if (!element->hasAttribute("joint"))
		RW_THROW("\"joint\" attribute missing.");
	std::string jointName = element->getAttributeValue("joint");

	rw::models::Joint::Ptr joint = (rw::models::Joint*) _stateStructure->findFrame(jointName);
	if (joint.isNull())
		RW_THROW("Joint \"" << jointName << "\" not found.");

	DHLinkCalibration::Ptr calibration = rw::common::ownedPtr(new DHLinkCalibration(joint));
	CalibrationParameterSet parameterSet = calibration->getParameterSet();

	if (!element->hasAttribute("a"))
		parameterSet(DHLinkCalibration::PARAMETER_A).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_A) = element->getAttributeValueAsDouble("a");

	if (!element->hasAttribute("b"))
		parameterSet(DHLinkCalibration::PARAMETER_B).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_B) = element->getAttributeValueAsDouble("b");

	if (!element->hasAttribute("d"))
		parameterSet(DHLinkCalibration::PARAMETER_D).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_D) = element->getAttributeValueAsDouble("d");

	if (!element->hasAttribute("alpha"))
		parameterSet(DHLinkCalibration::PARAMETER_ALPHA).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_ALPHA) = element->getAttributeValueAsDouble("alpha");

	if (!element->hasAttribute("beta"))
		parameterSet(DHLinkCalibration::PARAMETER_BETA).setEnabled(false);
	else
		parameterSet(DHLinkCalibration::PARAMETER_BETA) = element->getAttributeValueAsDouble("beta");

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

	if ( !elmRoot->hasChild("SerialDeviceCalibration") )
		RW_THROW("Element not found.");
	elmRoot = elmRoot->getChild("SerialDeviceCalibration");

	ElementReader elementReader(stateStructure, device);
	// Load base calibration.
	FixedFrameCalibration::Ptr baseCalibration;
	DOMElem::Ptr nodeBase = elmRoot->getChild("BaseCalibration");
	if (nodeBase!=NULL && nodeBase->hasChildren())
		baseCalibration = elementReader.readElement<FixedFrameCalibration::Ptr>( nodeBase );
	// Load end calibration.
	FixedFrameCalibration::Ptr endCalibration;
	DOMElem::Ptr nodeEnd = elmRoot->getChild("EndCalibration");
	if (nodeEnd!=NULL && nodeEnd->hasChildren() )
		endCalibration = elementReader.readElement<FixedFrameCalibration::Ptr>( nodeEnd );

	// Load link calibrations.
	CompositeCalibration<DHLinkCalibration>::Ptr compositeLinkCalibration = rw::common::ownedPtr(new CompositeCalibration<DHLinkCalibration>());
	DOMElem::Ptr nodeLinks = elmRoot->getChild("LinkCalibrations");
	if (nodeLinks!=NULL) {
		BOOST_FOREACH(DOMElem::Ptr child, nodeLinks->getChildren() ){
			DHLinkCalibration::Ptr linkCalibration = elementReader.readElement<DHLinkCalibration::Ptr>( child );
			compositeLinkCalibration->addCalibration(linkCalibration);
		}
	}

	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = rw::common::ownedPtr(new CompositeCalibration<JointEncoderCalibration>());
	SerialDeviceCalibration::Ptr calibration = rw::common::ownedPtr(new SerialDeviceCalibration(device, baseCalibration, endCalibration, compositeLinkCalibration, compositeJointCalibration));
	return calibration;
}
