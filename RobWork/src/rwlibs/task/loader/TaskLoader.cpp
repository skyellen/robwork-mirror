/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "TaskLoader.hpp"
#include <RobWorkConfig.hpp>
#include "DOMTaskLoader.hpp"
#ifdef RW_HAVE_XERCES
#include "XMLTaskLoader.hpp"
#endif

#include <rw/common/StringUtil.hpp>

using namespace rw::common;
using namespace rwlibs::task;

TaskLoader::Ptr TaskLoader::Factory::getTaskLoader(const std::string& format, const std::string& id) {
	TaskLoader::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts) {
		if(!ext->getProperties().has(format))
			continue;
		if (!id.empty()) {
			if (StringUtil::toUpper(ext->getId()) == StringUtil::toUpper(id))
				return ext->getObject().cast<const TaskLoader>()->clone();
		} else {
			return ext->getObject().cast<const TaskLoader>()->clone();
		}
	}
	if(StringUtil::toLower(format) == "xml") {
		if (id.empty())
			return rw::common::ownedPtr( new DOMTaskLoader() );
		else if (StringUtil::toUpper(id) == "DOM")
			return rw::common::ownedPtr( new DOMTaskLoader() );
#ifdef RW_HAVE_XERCES
		else if (StringUtil::toUpper(id) == "XERCES")
			return rw::common::ownedPtr( new DOMTaskLoader() );
#endif
	}
	return NULL;
}

bool TaskLoader::Factory::hasTaskLoader(const std::string& format) {
	if(StringUtil::toLower(format) == "xml")
		return true;

	TaskLoader::Factory ep;
	std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		if(!ext.getProperties().has(format))
			continue;
		return true;
	}
	return false;
}

std::vector<std::string> TaskLoader::Factory::getSupportedFormats() {
	std::set<std::string> ids;
	TaskLoader::Factory ep;
	std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
	ids.insert("xml");
	BOOST_FOREACH(Extension::Descriptor& ext, exts) {
		const PropertyMap& p = ext.getProperties();
		for (PropertyMap::iterator it = p.getProperties().first; it != p.getProperties().second; it++) {
			ids.insert(StringUtil::toLower((*it)->getIdentifier()));
		}
	}
	return std::vector<std::string>(ids.begin(),ids.end());
}
