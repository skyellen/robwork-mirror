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
#include "WorkCellFactory.hpp"

using namespace rw::loaders;
using namespace rw::models;
using namespace rw::common;

rw::common::Ptr<WorkCellLoader> WorkCellLoader::Factory::getWorkCellLoader(const std::string& format){
	using namespace rw::common;
	WorkCellLoader::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(!ext->getProperties().has(format))
			continue;
		// else try casting to ImageLoader
		WorkCellLoader::Ptr loader = ext->getObject().cast<WorkCellLoader>();
		return loader;
	}
	RW_THROW("No loader using that format exists...");
	return NULL;

}

WorkCell::Ptr WorkCellLoader::Factory::load(const std::string& file)
{
    const std::string ext2 = StringUtil::getFileExtension(file);
    const std::string ext = StringUtil::toUpper(ext2);
    try{
        if (ext == ".WU" || ext == ".WC" || ext == ".TAG" || ext == ".DEV") {
            return TULLoader::load(file);
        } else {
            return XMLRWLoader::load(file);
        }
    } catch (const std::exception& e){
        std::cout << "Exception: " << e.what() << std::endl;
    }

    // tjeck if any plugins support the file format
	WorkCellLoader::Ptr loader = WorkCellLoader::Factory::getWorkCellLoader(ext);
	if(loader!=NULL){
		try {
			WorkCell::Ptr wc = loader->loadWorkCell( file );
			return wc;
		} catch (...){
			Log::debugLog() << "Tried loading workcell with extension, but failed!\n";
		}
	}
    return NULL;
}
