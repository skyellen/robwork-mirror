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

#include "XmlCalibrationSaver.hpp"

#include <rw/common/DOMElem.hpp>
#include <rw/common/DOMParser.hpp>

#include <rw/loaders/dom/DOMBasisTypes.hpp>

#include "../JointEncoderCalibration.hpp"

using namespace rwlibs::calibration;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw;

class ElementCreator {
public:
	ElementCreator(DOMElem::Ptr root) :
		_root(root) {

	}

	template<class T>
	DOMElem::Ptr createElement(T object, DOMElem::Ptr parent);

private:
	DOMElem::Ptr _root;
};

template<>
DOMElem::Ptr ElementCreator::createElement<FixedFrameCalibration::Ptr>(
		FixedFrameCalibration::Ptr calibration,
		DOMElem::Ptr parent)
{
	DOMElem::Ptr element = parent->addChild("FixedFrameCalibration");
	element->addAttribute("frame")->setValue(calibration->getFrame()->getName());
	DOMElem::Ptr transformElement = element->addChild("Transform");
	transformElement->addAttribute("isPostCorrection")->setValue( calibration->isPostCorrection() );
	rw::math::Transform3D<> correction = calibration->getCorrectionTransform();
	DOMBasisTypes::write(correction, transformElement);

	return element;
}

template<>
DOMElem::Ptr ElementCreator::createElement<DHLinkCalibration::Ptr>(
		DHLinkCalibration::Ptr calibration,
		DOMElem::Ptr parent)
{
	DOMElem::Ptr element = parent->addChild("DHLinkCalibration");

	element->addAttribute("joint")->setValue( calibration->getJoint()->getName() );

	const CalibrationParameterSet parameterSet = calibration->getParameterSet();
	if (parameterSet(DHLinkCalibration::PARAMETER_A).isEnabled())
		element->addAttribute("a")->setValue( parameterSet(DHLinkCalibration::PARAMETER_A) );
	if (parameterSet(DHLinkCalibration::PARAMETER_B).isEnabled())
		element->addAttribute("b")->setValue( parameterSet(DHLinkCalibration::PARAMETER_B) );
	if (parameterSet(DHLinkCalibration::PARAMETER_D).isEnabled())
		element->addAttribute("d")->setValue( parameterSet(DHLinkCalibration::PARAMETER_D) );
	if (parameterSet(DHLinkCalibration::PARAMETER_ALPHA).isEnabled())
		element->addAttribute("alpha")->setValue( parameterSet(DHLinkCalibration::PARAMETER_ALPHA) );
	if (parameterSet(DHLinkCalibration::PARAMETER_BETA).isEnabled())
		element->addAttribute("beta")->setValue( parameterSet(DHLinkCalibration::PARAMETER_BETA) );
	if (parameterSet(DHLinkCalibration::PARAMETER_THETA).isEnabled())
		element->addAttribute("theta")->setValue(parameterSet(DHLinkCalibration::PARAMETER_THETA) );

	return element;
}

template<>
DOMElem::Ptr ElementCreator::createElement<JointEncoderCalibration::Ptr>(
		JointEncoderCalibration::Ptr calibration,
		DOMElem::Ptr parent)
{
	DOMElem::Ptr element = parent->addChild("DHLinkCalibration");

	element->addAttribute("joint")->setValue( calibration->getJoint()->getName() );

	const CalibrationParameterSet parameterSet = calibration->getParameterSet();
	if (parameterSet(JointEncoderCalibration::PARAMETER_TAU).isEnabled())
		element->addAttribute("tau")->setValue( parameterSet(JointEncoderCalibration::PARAMETER_TAU) );
	if (parameterSet(JointEncoderCalibration::PARAMETER_SIGMA).isEnabled())
		element->addAttribute("sigma")->setValue( parameterSet(JointEncoderCalibration::PARAMETER_SIGMA) );

	return element;
}


void createDOMDocument(DOMElem::Ptr rootDoc, SerialDeviceCalibration::Ptr calibration) {
	//rootElement->setName("SerialDeviceCalibration");
	DOMElem::Ptr rootElement = rootDoc->addChild("SerialDeviceCalibration");

	ElementCreator creator(rootElement);

	if (!calibration->getBaseCalibration().isNull()) {
		DOMElem::Ptr baseElement = rootElement->addChild("BaseCalibration");
		creator.createElement<FixedFrameCalibration::Ptr>(calibration->getBaseCalibration(), baseElement);
	}

	if (!calibration->getEndCalibration().isNull()) {
		DOMElem::Ptr endElement = rootElement->addChild("EndCalibration");
		creator.createElement<FixedFrameCalibration::Ptr>(calibration->getEndCalibration(), endElement);
	}

	CompositeCalibration<DHLinkCalibration>::Ptr compositeLinkCalibration = calibration->getCompositeLinkCalibration();
	const int linkCalibrationCount = compositeLinkCalibration->getCalibrationCount();
	if (linkCalibrationCount > 0) {
		DOMElem::Ptr linkCalibrationElement = rootElement->addChild( "LinkCalibrations" );
		for (int calibrationIndex = 0; calibrationIndex < linkCalibrationCount; calibrationIndex++) {
			DHLinkCalibration::Ptr linkCalibration = compositeLinkCalibration->getCalibration(calibrationIndex);
			//linkCalibrationElement->addChild("DHLinkCalibration");
			creator.createElement<DHLinkCalibration::Ptr>(linkCalibration, linkCalibrationElement);
		}
	}


	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = calibration->getCompositeJointCalibration();
	const int jointCalibrationCount = compositeJointCalibration->getCalibrationCount();
	if (jointCalibrationCount > 0) {

		DOMElem::Ptr jointCalibrationElement = rootElement->addChild( "JointCalibrations" );
		for (int calibrationIndex = 0; calibrationIndex < jointCalibrationCount; calibrationIndex++) {
			JointEncoderCalibration::Ptr jointCalibration = compositeJointCalibration->getCalibration(calibrationIndex);
			creator.createElement<JointEncoderCalibration::Ptr>(jointCalibration, jointCalibrationElement);
		}
	}

}

void XmlCalibrationSaver::save(SerialDeviceCalibration::Ptr calibration, std::string fileName) {
	DOMParser::Ptr doc = DOMParser::make();
	DOMElem::Ptr root = doc->getRootElement();

	createDOMDocument(root, calibration);

	// save to file
	doc->save( fileName );
}

void XmlCalibrationSaver::save(SerialDeviceCalibration::Ptr calibration, std::ostream& ostream) {
	DOMParser::Ptr doc = DOMParser::make();
	DOMElem::Ptr root = doc->getRootElement();

	createDOMDocument(root, calibration);

	// save to stream
	doc->save( ostream );
}


