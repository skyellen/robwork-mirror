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
#include "RenderSTL.hpp"
#include "Render3DS.hpp"
#include "RenderAC3D.hpp"
#include "RenderTriSoup.hpp"
#include "RenderGeometry.hpp"
#include "RenderIVG.hpp"

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
        ".TRI", ".AC", ".AC3D", ".3DS", ".IVG", ".STL", ".STLA", ".STLB"
    };

    const int extensionCount = sizeof(extensionsArray) / sizeof(extensionsArray[0]);

    const std::vector<std::string> extensions(
        extensionsArray, extensionsArray + extensionCount);
}

Drawable* DrawableFactory::getDrawable(const std::string& str)
{
    if (getCache().isInCache(str)) {
    	return new Drawable(getCache().get(str));
    }
    if (str[0] == '#') {
        return constructFromGeometry(str);
    }
    else {
        return loadDrawableFile(str);
    }
}

Drawable* DrawableFactory::constructFromGeometry(const std::string& str, bool useCache)
{
    if( useCache ){
    	if (getCache().isInCache(str))
    		return new Drawable(getCache().get(str));
    }
	std::auto_ptr<Geometry> geometry = GeometryFactory::getGeometry(str);
    Render *render = new RenderGeometry(geometry.release());

    if( useCache ) {
    	getCache().add(str, render);
    	return new Drawable(getCache().get(str));
    }

    return new Drawable(boost::shared_ptr<Render>(render));
}

DrawableFactory::Cache& DrawableFactory::getCache()
{
    static Cache cache;
	return cache;
}

Drawable* DrawableFactory::loadDrawableFile(const std::string &raw_filename)
{
    const std::string& filename = IOUtil::resolveFileName(raw_filename, extensions);
    const std::string& filetype =
        StringUtil::toUpper(StringUtil::getFileExtension(filename));

    // if the file does not exist then throw an exception
    if (filetype.empty()) {
        RW_THROW(
            "No file type known for file "
            << StringUtil::quote(raw_filename)
            << " that was resolved to file name "
            << filename);
    }

    if (getCache().isInCache(filename)) {
    	return new Drawable(getCache().get(filename));
    }

    // else check if the file has been loaded before
    if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
        Render *render = new RenderSTL(filename);
        getCache().add(filename, render);
        return new Drawable(getCache().get(filename));
    } else if (filetype == ".3DS") {
        Render *render = new Render3DS(filename);
        getCache().add(filename, render);
        boost::shared_ptr<Render> r = getCache().get(filename);
        return new Drawable(r);
    } else if (filetype == ".AC" || filetype == ".AC3D") {
        Render *render = new RenderAC3D(filename);
        getCache().add(filename, render);
        return new Drawable(getCache().get(filename));
    } else if (filetype == ".TRI") {
        Render *render = new RenderTriSoup(filename);
        getCache().add(filename, render);
        return new Drawable(getCache().get(filename));
    } else if (filetype == ".IVG") {
        Render *render = new RenderIVG(filename);
        getCache().add(filename, render);
        return new Drawable(getCache().get(filename));
	} else {
        RW_THROW(
            "Unknown extension "
            << StringUtil::quote(StringUtil::getFileExtension(filename))
            << " for file "
            << StringUtil::quote(raw_filename)
            << " that was resolved to file name "
            << filename);
    }

    RW_ASSERT(!"Impossible");
    return NULL; // To avoid a compiler warning.
}
