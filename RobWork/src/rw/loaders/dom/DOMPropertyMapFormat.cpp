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

#include "DOMPropertyMapFormat.hpp"

using namespace rw::loaders;

DOMPropertyMapFormat::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		idPropertyMap();
		idProperty();
		idPropertyName();
		idPropertyDescription();
		idPropertyValue();
		done = true;
	}
}

const DOMPropertyMapFormat::Initializer DOMPropertyMapFormat::initializer;

const std::string& DOMPropertyMapFormat::idPropertyMap() {
	static const std::string id("PropertyMap");
	return id;
}

const std::string& DOMPropertyMapFormat::idProperty() {
	static const std::string id("Property");
	return id;
}

const std::string& DOMPropertyMapFormat::idPropertyName() {
	static const std::string id("Name");
	return id;
}

const std::string& DOMPropertyMapFormat::idPropertyDescription() {
	static const std::string id("Description");
	return id;
}

const std::string& DOMPropertyMapFormat::idPropertyValue() {
	static const std::string id("Value");
	return id;
}
