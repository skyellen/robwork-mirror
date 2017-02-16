/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rwlibs/calibration/WorkCellCalibration.hpp>
#include <rwlibs/calibration/FixedFrameCalibration.hpp>
using namespace rwlibs::calibration;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw;

namespace { 
class ElementCreator {
public:
	ElementCreator(DOMElem::Ptr root)// :	_root(root) 
	{

	}

	template<class T>
	DOMElem::Ptr createElement(T object, DOMElem::Ptr parent);

private:
	//DOMElem::Ptr _root;
};

template<>
DOMElem::Ptr ElementCreator::createElement<FixedFrameCalibration::Ptr>(
		FixedFrameCalibration::Ptr calibration,
		DOMElem::Ptr parent)
{
	DOMElem::Ptr element = parent->addChild("FixedFrameCalibration");
	element->addAttribute("frame")->setValue(calibration->getFrame()->getName());
	DOMElem::Ptr transformElement = element->addChild("Transform3D");//Name does not matter, it is overwritten in the DOMBasisTypes::write anyway.
	rw::math::Transform3D<> corrected = calibration->getCorrectedTransform();
	DOMBasisTypes::write(corrected, transformElement, true);


	//Add the rotation in RPY just to make it easier for the user to see it.
	//DOMElem::Ptr rpyElement = element->addChild("RPY");//Name does not matter, it is overwritten in the DOMBasisTypes::write anyway.	
	//DOMBasisTypes::createRPY(RPY<>(correction.R()), element);



	return element;
}


void createDOMDocument(DOMElem::Ptr rootDoc, WorkCellCalibration::Ptr calibration) {
	//rootElement->setName("SerialDeviceCalibration");
	DOMElem::Ptr rootElement = rootDoc->addChild("WorkCellCalibration");

	ElementCreator creator(rootElement);

	for (Calibration::Ptr calib : calibration->getCalibrations())
	{
		calib.cast<FixedFrameCalibration>();
		if (calib.cast<FixedFrameCalibration>() != NULL) {
			DOMElem::Ptr element = rootElement->addChild("FixedFrameCalibration");
			creator.createElement<FixedFrameCalibration::Ptr>(calib.cast<FixedFrameCalibration>(), element);
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


