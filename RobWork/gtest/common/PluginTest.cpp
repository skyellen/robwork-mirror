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

#include <gtest/gtest.h>

#include "../TestEnvironment.hpp"

#include <rw/common/Plugin.hpp>
#include "TestPlugin.hpp"

using rw::common::Extension;
using rw::common::Plugin;

TEST(PluginTest, loadDirectPlugin) {
	const rw::common::Ptr<Plugin> plugin = Plugin::load(TestEnvironment::executableDir() + "libtest_plugin.rwplugin.so");

	EXPECT_EQ("TestPlugin", plugin->getId());
	EXPECT_EQ("Name of test plugin", plugin->getName());
	EXPECT_EQ("1.2.3", plugin->getVersion());

	const std::vector<std::string> ids = plugin->getExtensionPointIDs();
	EXPECT_EQ(3, ids.size());
	if (ids.size() >= 3) {
		EXPECT_EQ("ExtensionId1", ids[0]);
		EXPECT_EQ("ExtensionId2", ids[1]);
		EXPECT_EQ("ExtensionId3", ids[2]);
	}

	const std::vector<Extension::Descriptor> extDesc = plugin->getExtensionDescriptors();
	EXPECT_EQ(3, extDesc.size());
	if (ids.size() >= 3) {
		EXPECT_EQ("ExtensionId1", extDesc[0].id);
		EXPECT_EQ("ExtensionId2", extDesc[1].id);
		EXPECT_EQ("ExtensionId3", extDesc[2].id);
		EXPECT_EQ("", extDesc[0].name);
		EXPECT_EQ("", extDesc[1].name);
		EXPECT_EQ("", extDesc[2].name);
		EXPECT_EQ("rw_common-gtest.ExtensionPointA", extDesc[0].point);
		EXPECT_EQ("rw_common-gtest.ExtensionPointA", extDesc[1].point);
		EXPECT_EQ("rw_common-gtest.ExtensionPointB", extDesc[2].point);
	}

	Extension::Ptr ext1 = plugin->makeExtension("ExtensionId1");
	Extension::Ptr ext2 = plugin->makeExtension("ExtensionId2");
	Extension::Ptr ext3 = plugin->makeExtension("ExtensionId3");
	EXPECT_FALSE(ext1.isNull());
	EXPECT_FALSE(ext2.isNull());
	EXPECT_FALSE(ext3.isNull());

	EXPECT_EQ("ExtensionId1", ext1->getId());
	EXPECT_EQ("ExtensionId2", ext2->getId());
	EXPECT_EQ("ExtensionId3", ext3->getId());
	EXPECT_EQ("", ext1->getName());
	EXPECT_EQ("", ext2->getName());
	EXPECT_EQ("", ext3->getName());
	EXPECT_EQ("rw_common-gtest.ExtensionPointA", ext1->getPoint());
	EXPECT_EQ("rw_common-gtest.ExtensionPointA", ext2->getPoint());
	EXPECT_EQ("rw_common-gtest.ExtensionPointB", ext3->getPoint());
}

TEST(PluginTest, loadLazyPlugin) {
	const rw::common::Ptr<Plugin> plugin = Plugin::load(TestEnvironment::executableDir() + "test_plugin.rwplugin.xml");

	EXPECT_EQ("TestLazyPlugin", plugin->getId());
	EXPECT_EQ("Name of plugin for test.", plugin->getName());
	EXPECT_EQ("1.0", plugin->getVersion());

	const std::vector<std::string> ids = plugin->getExtensionPointIDs();
	EXPECT_EQ(3, ids.size());
	if (ids.size() >= 3) {
		EXPECT_EQ("ExtensionId1", ids[0]);
		EXPECT_EQ("ExtensionId2", ids[1]);
		EXPECT_EQ("ExtensionId3", ids[2]);
	}

	const std::vector<Extension::Descriptor> extDesc = plugin->getExtensionDescriptors();
	EXPECT_EQ(3, extDesc.size());
	if (ids.size() >= 3) {
		EXPECT_EQ("ExtensionId1", extDesc[0].id);
		EXPECT_EQ("ExtensionId2", extDesc[1].id);
		EXPECT_EQ("ExtensionId3", extDesc[2].id);
		EXPECT_EQ("Name of first extension.", extDesc[0].name);
		EXPECT_EQ("Name of second extension.", extDesc[1].name);
		EXPECT_EQ("Name of third extension.", extDesc[2].name);
		EXPECT_EQ("rw_common-gtest.ExtensionPointA", extDesc[0].point);
		EXPECT_EQ("rw_common-gtest.ExtensionPointA", extDesc[1].point);
		EXPECT_EQ("rw_common-gtest.ExtensionPointB", extDesc[2].point);
	}

	Extension::Ptr ext1 = plugin->makeExtension("ExtensionId1");
	Extension::Ptr ext2 = plugin->makeExtension("ExtensionId2");
	Extension::Ptr ext3 = plugin->makeExtension("ExtensionId3");
	EXPECT_FALSE(ext1.isNull());
	EXPECT_FALSE(ext2.isNull());
	EXPECT_FALSE(ext3.isNull());

	EXPECT_EQ("ExtensionId1", ext1->getId());
	EXPECT_EQ("ExtensionId2", ext2->getId());
	EXPECT_EQ("ExtensionId3", ext3->getId());
	EXPECT_EQ("", ext1->getName());
	EXPECT_EQ("", ext2->getName());
	EXPECT_EQ("", ext3->getName());
	EXPECT_EQ("rw_common-gtest.ExtensionPointA", ext1->getPoint());
	EXPECT_EQ("rw_common-gtest.ExtensionPointA", ext2->getPoint());
	EXPECT_EQ("rw_common-gtest.ExtensionPointB", ext3->getPoint());
}
