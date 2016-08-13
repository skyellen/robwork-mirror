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


#include "XMLTaskFormat.hpp"

#include <rw/loaders/xml/XercesUtils.hpp>

using namespace rwlibs::task;
using namespace rw::loaders;
using namespace xercesc;

XMLTaskFormat::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		//Small trick to make sure Xerces is initialized before we start using XMLString::transcode
		static XercesInitializer initializer;
		idQTask();
		idCartesianTask();
		idTargets();
		idEntities();
		idAugmentations();
		idQTarget();
		idCartesianTarget();
		idMotion();
		idAction();
		idEntityIndex();
		idEntityId();
		idTargetIdAttr();
		idMotionTypeAttr();
		idMotionStart();
		idMotionMid();
		idMotionEnd();
		idLinearMotion();
		idP2PMotion();
		idCircularMotion();
		idActionTypeAttr();
		done = true;
	}
}

const XMLTaskFormat::Initializer XMLTaskFormat::initializer;

const XMLCh* XMLTaskFormat::idQTask() {
	static const XMLStr id("QTask");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idCartesianTask() {
	static const XMLStr id("CartesianTask");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idTargets() {
	static const XMLStr id("Targets");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idEntities() {
	static const XMLStr id("Entities");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idAugmentations() {
	static const XMLStr id("Augmentations");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idQTarget() {
	static const XMLStr id("QTarget");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idCartesianTarget() {
	static const XMLStr id("CartesianTarget");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idMotion() {
	static const XMLStr id("Motion");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idAction() {
	static const XMLStr id("Action");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idEntityIndex() {
	static const XMLStr id("Index");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idEntityId() {
	static const XMLStr id("Id");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idTargetIdAttr() {
	static const XMLStr id("id");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idMotionTypeAttr() {
	static const XMLStr id("type");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idMotionStart() {
	static const XMLStr id("Start");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idMotionMid() {
	static const XMLStr id("Mid");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idMotionEnd() {
	static const XMLStr id("End");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idLinearMotion() {
	static const XMLStr id("Linear");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idP2PMotion() {
	static const XMLStr id("P2P");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idCircularMotion() {
	static const XMLStr id("Circular");
	return id.uni();
}

const XMLCh* XMLTaskFormat::idActionTypeAttr() {
	static const XMLStr id("type");
	return id.uni();
}
