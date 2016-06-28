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

using namespace rw::loaders;
using namespace xercesc;

//Small trick to make sure Xerces is initialized before we start using XMLString::transcode
const XercesInitializer XMLPathFormat::initializer;

const XMLCh* XMLPathFormat::QPathId = XMLString::transcode("QPath");
const XMLCh* XMLPathFormat::V3DPathId = XMLString::transcode("V3DPath");

const XMLCh* XMLPathFormat::R3DPathId = XMLString::transcode("R3DPath");
const XMLCh* XMLPathFormat::T3DPathId = XMLString::transcode("T3DPath");

const XMLCh* XMLPathFormat::StatePathId = XMLString::transcode("StatePath");

/** @brief Identifier for rw::trajectory::TimedQPath in the XML format  */
const XMLCh* XMLPathFormat::TimedQPathId = XMLString::transcode("TimedQPath");

/** @brief Identifier for rw::trajectory::TimedStatePath in the XML format  */
const XMLCh* XMLPathFormat::TimedStatePathId = XMLString::transcode("TimedStatePath");

/** @brief Identifier for rw::trajectory::TimedState in the XML format  */
const XMLCh* XMLPathFormat::TimedStateId = XMLString::transcode("TimedState");

/** @brief Identifier for rw::trajectory::TimedQ in the XML format  */
const XMLCh* XMLPathFormat::TimedQId = XMLString::transcode("TimedQ");

/** @brief Identifier for time attribute used for rw::trajectory::TimedQPath and rw::trajectory::TimedStatePath in the XML format  */
const XMLCh* XMLPathFormat::TimeId = XMLString::transcode("Time"); 

const int* XMLPathFormat::myTestVar = NULL;

namespace {
	void test() {
		std::cout<<XMLPathFormat::TimedQId<<std::endl;
	}

}
