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

XMLPropertyFormat::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		static XercesInitializer initializer;
	    idPropertyMap();
	    idProperty();
	    idPropertyName();
	    idPropertyDescription();
	    idPropertyType();
	    idPropertyValue();
		done = true;
	}
}

const XMLPropertyFormat::Initializer XMLPropertyFormat::initializer;

const XMLCh* XMLPropertyFormat::idPropertyMap() {
	static const XMLStr id("PropertyMap");
	return id.uni();
}

const XMLCh* XMLPropertyFormat::idProperty() {
	static const XMLStr id("Property");
	return id.uni();
}

const XMLCh* XMLPropertyFormat::idPropertyName() {
	static const XMLStr id("Name");
	return id.uni();
}

const XMLCh* XMLPropertyFormat::idPropertyDescription() {
	static const XMLStr id("Description");
	return id.uni();
}

const XMLCh* XMLPropertyFormat::idPropertyType() {
	static const XMLStr id("Type");
	return id.uni();
}

const XMLCh* XMLPropertyFormat::idPropertyValue() {
	static const XMLStr id("Value");
	return id.uni();
}
