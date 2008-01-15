/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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
#include <fstream>
#include <cctype>

#include "DrawableFactory.hpp"
#include "DrawableSTL.hpp"
#include "Drawable3DS.hpp"
#include "DrawableAC3D.hpp"
#include "DrawableTriSoup.hpp"
#include "DrawableGeometry.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <rw/common/macros.hpp>

#include <rw/geometry/GeometryFactory.hpp>

#include <string>
#include <istream>
#include <sstream>

using namespace rw;
using namespace rwlibs::drawable;
using namespace rw::common;
using namespace rw::geometry;

namespace
{
    const std::string extensionsArray[] = {
        ".TRI", ".AC", ".AC3D", ".3DS", ".STL", ".STLA", ".STLB"
    };

    const int extensionCount = sizeof(extensionsArray) / sizeof(extensionsArray[0]);

    const std::vector<std::string> extensions(
        extensionsArray, extensionsArray + extensionCount);
}

Drawable* DrawableFactory::GetDrawable(const std::string& str)
{
    if (str[0] == '#') {
        return ConstructFromGeometry(str);
    }
    else {
        return LoadDrawableFile(str);
    }
}

Drawable* DrawableFactory::ConstructFromGeometry(const std::string& str)
{
    Geometry* geometry = GeometryFactory::GetGeometry(str);
    return new DrawableGeometry(geometry);
}

Drawable* DrawableFactory::LoadDrawableFile(const std::string &raw_filename)
{
    const std::string& filename = IOUtil::ResolveFileName(raw_filename, extensions);
    const std::string& filetype =
        StringUtil::ToUpper(StringUtil::GetFileExtension(filename));

    if (!filetype.empty()) {
        if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
            return new DrawableSTL(filename);
        } else if (filetype == ".3DS") {
            return new Drawable3DS(filename);
        } else if (filetype == ".AC" || filetype == ".AC3D") {
            return new DrawableAC3D(filename);
        } else if (filetype == ".TRI") {
            return new DrawableTriSoup(filename);
        } else {
            RW_THROW(
                "Unknown extension "
                << StringUtil::Quote(StringUtil::GetFileExtension(filename))
                << " for file "
                << StringUtil::Quote(raw_filename)
                << " that was resolved to file name "
                << filename);
        }
    } else {
        RW_THROW(
            "No file type known for file "
            << StringUtil::Quote(raw_filename)
            << " that was resolved to file name "
            << filename);
    }

    RW_ASSERT(!"Impossible");
    return NULL; // To avoid a compiler warning.
}
