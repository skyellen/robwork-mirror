/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "TestPlugin.hpp"

using rw::common::Extension;
using rw::common::Plugin;

RW_ADD_PLUGIN(TestPlugin)

TestPlugin::TestPlugin():
	Plugin("TestPlugin", "Name of test plugin", "1.2.3")
{
}

TestPlugin::~TestPlugin() {
}

std::vector<Extension::Descriptor> TestPlugin::getExtensionDescriptors() {
	std::vector<Extension::Descriptor> desc;
	desc.push_back(Extension::Descriptor("ExtensionId1","rw_common-gtest.ExtensionPointA"));
	desc.push_back(Extension::Descriptor("ExtensionId2","rw_common-gtest.ExtensionPointA"));
	desc.push_back(Extension::Descriptor("ExtensionId3","rw_common-gtest.ExtensionPointB"));
	return desc;
}

Extension::Ptr TestPlugin::makeExtension(const std::string& id) {
	if (id == "ExtensionId1")
		return ownedPtr(new Extension("ExtensionId1","rw_common-gtest.ExtensionPointA",this));
	else if (id == "ExtensionId2")
		return ownedPtr(new Extension("ExtensionId2","rw_common-gtest.ExtensionPointA",this));
	else if (id == "ExtensionId3")
		return ownedPtr(new Extension("ExtensionId3","rw_common-gtest.ExtensionPointB",this));
	return NULL;
}

std::vector<std::string> TestPlugin::getExtensionPointIDs() {
	std::vector<std::string> ids;
	ids.push_back("ExtensionId1");
	ids.push_back("ExtensionId2");
	ids.push_back("ExtensionId3");
	return ids;
}
