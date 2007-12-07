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

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>

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
}

bool FaceArrayFactory::GetFaceArray(const std::string& str,
                                   std::vector<Face<float> >& result) {
    if (str[0] == '#')
        return ConstructFromGeometry(str, result);
    else 
        return LoadFaceArrayFile(str, result);
}


bool FaceArrayFactory::LoadFaceArrayFile(const std::string &raw_filename,
                                         std::vector<Face<float> >& result)
{
    try {
        const std::string& filename =
            IOUtil::ResolveFileName(raw_filename, extensions);
        const std::string& filetype =
            StringUtil::ToUpper(StringUtil::GetFileExtension(filename));
        
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


bool FaceArrayFactory::ConstructFromGeometry(const std::string& str,
                                             std::vector<Face<float> >& result) {
    Geometry* geometry = GeometryFactory::GetGeometry(str);
    const std::list<Face<float> >& faces = geometry->getFaces();
    for (std::list<Face<float> >::const_iterator it = faces.begin(); it != faces.end(); ++it) {
        result.push_back(*it);
    }
    delete geometry;
    return true;
}
