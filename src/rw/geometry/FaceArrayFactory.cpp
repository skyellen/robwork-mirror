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


#include "FaceArrayFactory.hpp"

#include <fstream>
#include <cctype>
#include <list>

#include "GeometrySTL.hpp"
#include "GeometryFactory.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>

#include <sys/types.h>
#include <sys/stat.h>

using namespace rw::geometry;
using namespace rw::common;

namespace
{
    const std::string extensionsArray[] = {
        ".STL", ".STLA", ".STLB"
    };

    const int extensionCount = sizeof(extensionsArray) / sizeof(extensionsArray[0]);

    const std::vector<std::string> extensions(
        extensionsArray, extensionsArray + extensionCount);

    std::string getLastModifiedStr(const std::string& file){
        struct stat status;
        stat(file.c_str(), &status);
        //std::cout << "LAST MODIFIED DATE: " << status.st_mtime << std::endl;
        std::stringstream sstr;
        sstr<< status.st_mtime;
        return sstr.str();
    }
}

bool FaceArrayFactory::getFaceArray(
    const std::string& str,
    std::vector<Face<float> >& result)
{
    if (str[0] == '#')
        return constructFromGeometry(str, result);
    else
        return loadFaceArrayFile(str, result);
}


bool FaceArrayFactory::loadFaceArrayFile(
    const std::string &raw_filename,
    std::vector<Face<float> >& result)
{
    try {
        const std::string& filename =
            IOUtil::resolveFileName(raw_filename, extensions);
        const std::string& filetype =
            StringUtil::toUpper(StringUtil::getFileExtension(filename));

        if (!filetype.empty()) {

            std::string moddate = getLastModifiedStr(filename);

            if( getCache().isInCache( filename, moddate) ){
                result = *getCache().get(filename);
                //std::cout << "Cache hit!" << std::endl;
                return true;
            }

            if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
                std::vector<Face<float> > *resulttmp = new std::vector<Face<float> >();
                GeometrySTL::load(filename, *resulttmp);
                result = *resulttmp;
                getCache().add(filename, resulttmp, moddate);
                return true;
            } else {
                return false;
            }
        } else
            return false;
    }
    catch (const Exception& exp) {
		RW_WARN(exp.getMessage().getText());
        return false;
    }
}

FaceArrayFactory::FactoryCache& FaceArrayFactory::getCache()
{
    static FactoryCache cache;
	return cache;
}

bool FaceArrayFactory::constructFromGeometry(
    const std::string& str,
    std::vector<Face<float> >& result)
{
    std::auto_ptr<Geometry> geometry = GeometryFactory::getGeometry(str);
    result.insert(
        result.end(),
        geometry->getFaces().begin(),
        geometry->getFaces().end());

    return true;
}
