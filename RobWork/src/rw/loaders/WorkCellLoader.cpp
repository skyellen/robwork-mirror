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


#include "WorkCellLoader.hpp"

#include <rw/loaders/tul/TULLoader.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw::common;
using namespace rw::loaders;
using rw::models::WorkCell;

WorkCellLoader::Ptr WorkCellLoader::Factory::getWorkCellLoader(const std::string& format){
	WorkCellLoader::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(!ext->getProperties().has(format))
			continue;
		// else try casting to WorkCellLoader
		WorkCellLoader::Ptr loader = ext->getObject().cast<WorkCellLoader>();
		if (!loader.isNull())
			return loader;
	}

	// Fallback to default formats
	if (format == ".WU" || format == ".WC" || format == ".TAG" || format == ".DEV") {
		return ownedPtr(new TULLoader());
	} else {
		return ownedPtr(new XMLRWLoader());
	}
}

WorkCell::Ptr WorkCellLoader::Factory::load(const std::string& file) {
    const std::string ext2 = StringUtil::getFileExtension(file);
    const std::string ext = StringUtil::toUpper(ext2);
	const WorkCellLoader::Ptr loader = getWorkCellLoader(ext);
	try {
		return loader->loadWorkCell(file);
	} catch (const std::exception&){
		Log::debugLog() << "Tried loading workcell with extension, but failed!\n";
	}
    return NULL;
}
