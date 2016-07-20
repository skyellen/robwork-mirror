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

/*
 * XMLPropertyFormat.cpp
 *
 *  Created on: Jan 5, 2009
 *      Author: lpe
 */

#include "XMLPropertyFormat.hpp"
#include "XercesUtils.hpp"

using namespace rw::loaders;
using namespace xercesc;

const XercesInitializer XMLPropertyFormat::initializer;

/** @brief Identifier for rw::common::PropertyMap in the XML format  */
const XMLCh* XMLPropertyFormat::PropertyMapId = XMLString::transcode("PropertyMap");

/** @brief Identifier for rw::common::Property in the XML format  */
const XMLCh* XMLPropertyFormat::PropertyId = XMLString::transcode("Property");

/** @brief Identifier for the name of a rw::common::Property */
const XMLCh* XMLPropertyFormat::PropertyNameId = XMLString::transcode("Name");

/** @brief Identifier for the description of a rw::common::Property */
const XMLCh* XMLPropertyFormat::PropertyDescriptionId = XMLString::transcode("Description");

/** @brief Identifier for the type of a rw::common::Property */
const XMLCh* XMLPropertyFormat::PropertyTypeId = XMLString::transcode("Type");

/** @brief Identifier for the value of a rw::common::Property */
const XMLCh* XMLPropertyFormat::PropertyValueId = XMLString::transcode("Value");
