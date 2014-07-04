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

namespace { 
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
	DOMElem::Ptr transformElement = element->addChild("Transform3D");//Name does not matter, it is overwritten in the DOMBasisTypes::write anyway.
//	transformElement->addAttribute("isPostCorrection")->setValue( calibration->isPostCorrection() );
	rw::math::Transform3D<> correction = calibration->getCorrectionTransform();
	DOMBasisTypes::write(correction, transformElement, true);

	return element;
}

template<>
DOMElem::Ptr ElementCreator::createElement<ParallelAxisDHCalibration::Ptr>(
		ParallelAxisDHCalibration::Ptr calibration,
		DOMElem::Ptr parent)
{
	DOMElem::Ptr element = parent->addChild("ParallelAxisCalibration");

	element->addAttribute("joint")->setValue( calibration->getJoint()->getName() );

	const CalibrationParameterSet parameterSet = calibration->getParameterSet();
	if (parameterSet(ParallelAxisDHCalibration::PARAMETER_A).isEnabled())
		element->addAttribute("a")->setValue( parameterSet(ParallelAxisDHCalibration::PARAMETER_A) );
	if (parameterSet(ParallelAxisDHCalibration::PARAMETER_B).isEnabled())
		element->addAttribute("b")->setValue( parameterSet(ParallelAxisDHCalibration::PARAMETER_B) );
	if (parameterSet(ParallelAxisDHCalibration::PARAMETER_D).isEnabled())
		element->addAttribute("d")->setValue( parameterSet(ParallelAxisDHCalibration::PARAMETER_D) );
	if (parameterSet(ParallelAxisDHCalibration::PARAMETER_ALPHA).isEnabled())
		element->addAttribute("alpha")->setValue( parameterSet(ParallelAxisDHCalibration::PARAMETER_ALPHA) );
	if (parameterSet(ParallelAxisDHCalibration::PARAMETER_BETA).isEnabled())
		element->addAttribute("beta")->setValue( parameterSet(ParallelAxisDHCalibration::PARAMETER_BETA) );
	if (parameterSet(ParallelAxisDHCalibration::PARAMETER_THETA).isEnabled())
		element->addAttribute("theta")->setValue(parameterSet(ParallelAxisDHCalibration::PARAMETER_THETA) );

	return element;
}

template<>
DOMElem::Ptr ElementCreator::createElement<JointEncoderCalibration::Ptr>(
		JointEncoderCalibration::Ptr calibration,
		DOMElem::Ptr parent)
{
	DOMElem::Ptr element = parent->addChild("JointEncoderCalibration");
	
	element->addAttribute("joint")->setValue( calibration->getJoint()->getName() );

	const CalibrationParameterSet parameterSet = calibration->getParameterSet();
	if (parameterSet(JointEncoderCalibration::PARAMETER_TAU).isEnabled())
		element->addAttribute("tau")->setValue( parameterSet(JointEncoderCalibration::PARAMETER_TAU) );
	if (parameterSet(JointEncoderCalibration::PARAMETER_SIGMA).isEnabled())
		element->addAttribute("sigma")->setValue( parameterSet(JointEncoderCalibration::PARAMETER_SIGMA) );

	return element;
}


void createDOMDocument(DOMElem::Ptr rootDoc, WorkCellCalibration::Ptr calibration) {
	//rootElement->setName("SerialDeviceCalibration");
	DOMElem::Ptr rootElement = rootDoc->addChild("WorkCellCalibration");

	ElementCreator creator(rootElement);

	CompositeCalibration<FixedFrameCalibration>::Ptr fixedFrameCalibrations = calibration->getFixedFrameCalibrations();
	for (int i = 0; i<fixedFrameCalibrations->getCalibrationCount(); i++) {
		FixedFrameCalibration::Ptr ffc = fixedFrameCalibrations->getCalibration(i);
		DOMElem::Ptr element = rootElement->addChild("FixedFrameCalibration");
		creator.createElement<FixedFrameCalibration::Ptr>(ffc, element);

	}

	/*if (!calibration->getBaseCalibration().isNull()) {
		DOMElem::Ptr element = rootElement->addChild("FixedFrameCalibration");
		creator.createElement<FixedFrameCalibration::Ptr>(calibration->getBaseCalibration(), element);
	}

	if (!calibration->getEndCalibration().isNull()) {
		DOMElem::Ptr endElement = rootElement->addChild("FixedFrameCalibration");
		creator.createElement<FixedFrameCalibration::Ptr>(calibration->getEndCalibration(), endElement);
	}*/

	CompositeCalibration<ParallelAxisDHCalibration>::Ptr compositeLinkCalibration = calibration->getCompositeLinkCalibration();
	const int linkCalibrationCount = compositeLinkCalibration->getCalibrationCount();
	if (linkCalibrationCount > 0) {
		DOMElem::Ptr linkCalibrationElement = rootElement->addChild( "LinkCalibrations" );
		for (int calibrationIndex = 0; calibrationIndex < linkCalibrationCount; calibrationIndex++) {
			ParallelAxisDHCalibration::Ptr linkCalibration = compositeLinkCalibration->getCalibration(calibrationIndex);
			//linkCalibrationElement->addChild("DHLinkCalibration");
			creator.createElement<ParallelAxisDHCalibration::Ptr>(linkCalibration, linkCalibrationElement);
		}
	}


	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = calibration->getCompositeJointCalibration();
	const int jointCalibrationCount = compositeJointCalibration->getCalibrationCount();
	if (jointCalibrationCount > 0) {
		DOMElem::Ptr jointCalibrationElement = rootElement->addChild( "JointCalibrations" );
		jointCalibrationElement->addAttribute("device")->setValue(compositeJointCalibration->getCalibration(0)->getDevice()->getName());
		for (int calibrationIndex = 0; calibrationIndex < jointCalibrationCount; calibrationIndex++) {
			JointEncoderCalibration::Ptr jointCalibration = compositeJointCalibration->getCalibration(calibrationIndex);
			creator.createElement<JointEncoderCalibration::Ptr>(jointCalibration, jointCalibrationElement);
		}
	}

}

} //End anonymous namespace


void XmlCalibrationSaver::save(WorkCellCalibration::Ptr calibration, std::string fileName) {
	DOMParser::Ptr doc = DOMParser::make();
	DOMElem::Ptr root = doc->getRootElement();

	createDOMDocument(root, calibration);

	// save to file
	doc->save( fileName );
}

void XmlCalibrationSaver::save(WorkCellCalibration::Ptr calibration, std::ostream& ostream) {
	DOMParser::Ptr doc = DOMParser::make();
	DOMElem::Ptr root = doc->getRootElement();

	createDOMDocument(root, calibration);

	// save to stream
	doc->save( ostream );
}


