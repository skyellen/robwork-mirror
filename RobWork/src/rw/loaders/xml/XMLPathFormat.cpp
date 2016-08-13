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

#include "XMLPathFormat.hpp"

#include "XercesUtils.hpp"

using namespace rw::loaders;
using namespace xercesc;

XMLPathFormat::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		//Small trick to make sure Xerces is initialized before we start using XMLString::transcode
		static XercesInitializer initializer;
		idQPath();
		idV3DPath();
		idR3DPath();
		idT3DPath();
		idStatePath();
		idTimedQPath();
		idTimedState();
		idTimedQ();
		idTimedStatePath();
		idTime();
		done = true;
	}
}

const XMLPathFormat::Initializer XMLPathFormat::initializer;

const XMLCh* XMLPathFormat::idQPath() {
	static const XMLStr id("QPath");
	return id.uni();
}

const XMLCh* XMLPathFormat::idV3DPath() {
	static const XMLStr id("V3DPath");
	return id.uni();
}

const XMLCh* XMLPathFormat::idR3DPath() {
	static const XMLStr id("R3DPath");
	return id.uni();
}

const XMLCh* XMLPathFormat::idT3DPath() {
	static const XMLStr id("T3DPath");
	return id.uni();
}

const XMLCh* XMLPathFormat::idStatePath() {
	static const XMLStr id("StatePath");
	return id.uni();
}

const XMLCh* XMLPathFormat::idTimedQPath() {
	static const XMLStr id("TimedQPath");
	return id.uni();
}

const XMLCh* XMLPathFormat::idTimedState() {
	static const XMLStr id("TimedState");
	return id.uni();
}

const XMLCh* XMLPathFormat::idTimedQ() {
	static const XMLStr id("TimedQ");
	return id.uni();
}

const XMLCh* XMLPathFormat::idTimedStatePath() {
	static const XMLStr id("TimedStatePath");
	return id.uni();
}

const XMLCh* XMLPathFormat::idTime() {
	static const XMLStr id("Time");
	return id.uni();
}
