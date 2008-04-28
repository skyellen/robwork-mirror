/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "FaceArrayFactory.hpp"

#include <fstream>
#include <cctype>
#include <list>

#include "GeometrySTL.hpp"
#include "GeometryFactory.hpp"

#include <rw/common/Cache.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>

using namespace rw::geometry;
using namespace rw::common;

FaceArrayFactory::Cache cache;

namespace
{
    const std::string extensionsArray[] = {
        ".STL", ".STLA", ".STLB"
    };

    const int extensionCount = sizeof(extensionsArray) / sizeof(extensionsArray[0]);

    const std::vector<std::string> extensions(
        extensionsArray, extensionsArray + extensionCount);
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
        
        /*if( getCache().isInCache(filename) ){
        	return new GeometryMesh( getCache().get(filename) );
        }*/
        
        if (!filetype.empty()) {
            if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
                GeometrySTL::ReadSTL(filename, result);
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

FaceArrayFactory::Cache& FaceArrayFactory::getCache()
{
    static Cache cache;
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
