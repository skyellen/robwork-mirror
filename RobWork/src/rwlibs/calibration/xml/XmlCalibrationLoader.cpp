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
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>

using namespace rwlibs::calibration;
using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw;

namespace {



	FixedFrameCalibration::Ptr readFixedFrameCalibration(DOMElem::Ptr felement, WorkCell::Ptr workcell) {
		DOMElem::Ptr element = felement->getChild("FixedFrameCalibration");

		if (!element->hasAttribute("frame"))
			RW_THROW("\"frame\" attribute missing.");
		std::string frameName = element->getAttributeValue("frame");
		rw::kinematics::Frame* frame = workcell->findFrame(frameName);
		rw::kinematics::FixedFrame::Ptr fixedFrame = rw::kinematics::Frame::Ptr(frame).cast<rw::kinematics::FixedFrame>();
		if (fixedFrame.isNull())
			RW_THROW("Frame \"" << frameName << "\" not found.");

		DOMElem::Ptr transformElement = element->getChild("Transform3D");
		if (transformElement == NULL)
			RW_THROW("\"Transform3D\" element not found");

		Transform3D<> t3d = DOMBasisTypes::readTransform3D(transformElement, false);

		//if (!transformElement->hasAttribute("isPostCorrection"))
		//	RW_THROW("\"isPostCorrection\" attribute missing.");
		//bool isPostCorrection = transformElement->getAttributeValueAsBool("isPostCorrection");

		return rw::common::ownedPtr(new FixedFrameCalibration(fixedFrame, t3d));
	}
	

	ParallelAxisDHCalibration::Ptr readDHLinkCalibration(DOMElem::Ptr element, WorkCell::Ptr workcell) {
		if (!element->hasAttribute("joint"))
			RW_THROW("\"joint\" attribute missing.");
		std::string jointName = element->getAttributeValue("joint");

		rw::models::Joint::Ptr joint = dynamic_cast<rw::models::Joint*>( workcell->findFrame(jointName) );
		if (joint.isNull())
			RW_THROW("Joint \"" << jointName << "\" not found.");

		ParallelAxisDHCalibration::Ptr calibration = rw::common::ownedPtr(new ParallelAxisDHCalibration(joint));
		CalibrationParameterSet parameterSet = calibration->getParameterSet();

		if (!element->hasAttribute("a"))
			parameterSet(ParallelAxisDHCalibration::PARAMETER_A).setEnabled(false);
		else
			parameterSet(ParallelAxisDHCalibration::PARAMETER_A) = element->getAttributeValueAsDouble("a");

		if (!element->hasAttribute("b"))
			parameterSet(ParallelAxisDHCalibration::PARAMETER_B).setEnabled(false);
		else
			parameterSet(ParallelAxisDHCalibration::PARAMETER_B) = element->getAttributeValueAsDouble("b");

		if (!element->hasAttribute("d"))
			parameterSet(ParallelAxisDHCalibration::PARAMETER_D).setEnabled(false);
		else
			parameterSet(ParallelAxisDHCalibration::PARAMETER_D) = element->getAttributeValueAsDouble("d");

		if (!element->hasAttribute("alpha"))
			parameterSet(ParallelAxisDHCalibration::PARAMETER_ALPHA).setEnabled(false);
		else
			parameterSet(ParallelAxisDHCalibration::PARAMETER_ALPHA) = element->getAttributeValueAsDouble("alpha");

		if (!element->hasAttribute("beta"))
			parameterSet(ParallelAxisDHCalibration::PARAMETER_BETA).setEnabled(false);
		else
			parameterSet(ParallelAxisDHCalibration::PARAMETER_BETA) = element->getAttributeValueAsDouble("beta");

		if (!element->hasAttribute("theta"))
			parameterSet(ParallelAxisDHCalibration::PARAMETER_THETA).setEnabled(false);
		else
			parameterSet(ParallelAxisDHCalibration::PARAMETER_THETA) = element->getAttributeValueAsDouble("theta");

		calibration->setParameterSet(parameterSet);

		return calibration;
	}


	JointEncoderCalibration::Ptr readDHJointCalibration(DOMElem::Ptr element, WorkCell::Ptr workcell, SerialDevice::Ptr device) {
			if (!element->hasAttribute("joint"))
				RW_THROW("\"joint\" attribute missing.");
			std::string jointName = element->getAttributeValue("joint");


			rw::models::Joint::Ptr joint = dynamic_cast<rw::models::Joint*>( workcell->findFrame(jointName) );
			if (joint.isNull())
				RW_THROW("Joint \"" << jointName << "\" not found.");

			JointEncoderCalibration::Ptr calibration = rw::common::ownedPtr(new JointEncoderCalibration(device, joint));
			CalibrationParameterSet parameterSet = calibration->getParameterSet();

			if (!element->hasAttribute("tau"))
				parameterSet(JointEncoderCalibration::PARAMETER_TAU).setEnabled(false);
			else
				parameterSet(JointEncoderCalibration::PARAMETER_TAU) = element->getAttributeValueAsDouble("tau");

			if (!element->hasAttribute("sigma"))
				parameterSet(JointEncoderCalibration::PARAMETER_SIGMA).setEnabled(false);
			else
				parameterSet(JointEncoderCalibration::PARAMETER_SIGMA) = element->getAttributeValueAsDouble("sigma");

			calibration->setParameterSet(parameterSet);

			return calibration;
		}

}

WorkCellCalibration::Ptr XmlCalibrationLoader::load(rw::models::WorkCell::Ptr workcell,
													std::string fileName)
{
	DOMParser::Ptr parser = DOMParser::make();

	parser->load(fileName);

	DOMElem::Ptr elmRoot = parser->getRootElement();

	if ( !elmRoot->hasChild("WorkCellCalibration") )
		RW_THROW("Element not found.");
	elmRoot = elmRoot->getChild("WorkCellCalibration");
	//FixedFrameCalibration::Ptr baseCalibration;
	//FixedFrameCalibration::Ptr endCalibration;
	CompositeCalibration<FixedFrameCalibration>::Ptr compositeFixedFrameCalibration = rw::common::ownedPtr(new CompositeCalibration<FixedFrameCalibration>());
	CompositeCalibration<ParallelAxisDHCalibration>::Ptr compositeLinkCalibration = rw::common::ownedPtr(new CompositeCalibration<ParallelAxisDHCalibration>());
	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = rw::common::ownedPtr(new CompositeCalibration<JointEncoderCalibration>());
	BOOST_FOREACH(DOMElem::Ptr child, elmRoot->getChildren()) 
	{
		if (child->getName() == "FixedFrameCalibration") {
			FixedFrameCalibration::Ptr ffCalibration = readFixedFrameCalibration( child, workcell );
			compositeFixedFrameCalibration->addCalibration(ffCalibration);
		}
		//ElementReader elementReader(, device);
		// Load base calibration.
		
		/*DOMElem::Ptr nodeBase = elmRoot->getChild("FixedFrameCalibration");
		if (nodeBase!=NULL && nodeBase->hasChildren())
			baseCalibration = readFixedFrameCalibration( nodeBase, workcell );
		// Load end calibration.
		
		DOMElem::Ptr nodeEnd = elmRoot->getChild("FixedFrameCalibration");
		if (nodeEnd!=NULL && nodeEnd->hasChildren() )
			endCalibration = readFixedFrameCalibration( nodeEnd, workcell );
			*/
		// Load link calibrations.
		
		DOMElem::Ptr nodeLinks = elmRoot->getChild("LinkCalibrations");

		if (nodeLinks!=NULL) {
			BOOST_FOREACH(DOMElem::Ptr child, nodeLinks->getChildren() ){
				ParallelAxisDHCalibration::Ptr linkCalibration = readDHLinkCalibration( child, workcell );
				compositeLinkCalibration->addCalibration(linkCalibration);
			}
		}

		
		DOMElem::Ptr nodeJoints = elmRoot->getChild("JointCalibrations");

		if (!nodeJoints->hasAttribute("device"))
			RW_THROW("\"device\" attribute missing.");
		const std::string deviceName = nodeJoints->getAttributeValue("device");
		SerialDevice::Ptr device  = workcell->findDevice<SerialDevice>(deviceName);
		if (device.isNull())
			RW_THROW("Unable to find device '"<<deviceName<<"' in work cell");


		if (nodeJoints!=NULL) {
			BOOST_FOREACH(DOMElem::Ptr child, nodeJoints->getChildren() ){
				JointEncoderCalibration::Ptr jointCalibration = readDHJointCalibration( child, workcell, device);
				compositeJointCalibration->addCalibration(jointCalibration);
			}
		}
	} //End BOOST_FOREACH


	WorkCellCalibration::Ptr calibration = rw::common::ownedPtr(new WorkCellCalibration(compositeFixedFrameCalibration, /*baseCalibration, endCalibration, */compositeLinkCalibration, compositeJointCalibration));
	return calibration;
}
