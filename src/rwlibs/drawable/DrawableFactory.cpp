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

#include <fstream>
#include <cctype>

#include "DrawableFactory.hpp"
#include "RenderSTL.hpp"
#include "Render3DS.hpp"
#include "RenderAC3D.hpp"
#include "RenderTriSoup.hpp"
#include "RenderGeometry.hpp"
#include "RenderOBJ.hpp"
#include "RenderIVG.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <rw/common/macros.hpp>

#include <rw/geometry/GeometryFactory.hpp>

#include <string>
#include <istream>
#include <sstream>

#include <sys/types.h>
#include <sys/stat.h>

using namespace rw;
using namespace rwlibs::drawable;
using namespace rw::common;
using namespace rw::geometry;

namespace
{
    const std::string extensionsArray[] = {
        ".TRI", ".AC", ".AC3D", ".3DS", ".OBJ", ".IVG", ".STL", ".STLA", ".STLB"
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

Drawable* DrawableFactory::getDrawable(const std::string& str)
{
    if (getCache().isInCache(str,"")) {
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
    	if (getCache().isInCache(str,""))
    		return new Drawable(getCache().get(str));
    }
	std::auto_ptr<Geometry> geometry = GeometryFactory::getGeometry(str);
    Render *render = new RenderGeometry(geometry.release());

    if( useCache ) {
    	getCache().add(str, render, "");
    	return new Drawable(getCache().get(str));
    }

    return new Drawable(boost::shared_ptr<Render>(render));
}

DrawableFactory::FactoryCache& DrawableFactory::getCache()
{
    static FactoryCache cache;
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

    std::string moddate = getLastModifiedStr(filename);
    if ( getCache().isInCache(filename, moddate) ) {
    	return new Drawable(getCache().get(filename));
    }

    // else check if the file has been loaded before
    if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
        Render *render = new RenderSTL(filename);
        getCache().add(filename, render, moddate);
        return new Drawable(getCache().get(filename));
    } else if (filetype == ".3DS") {
        Render *render = new Render3DS(filename);
        getCache().add(filename, render, moddate);
        rw::common::Ptr<Render> r = getCache().get(filename);
        return new Drawable(r);
    } else if (filetype == ".AC" || filetype == ".AC3D") {
        Render *render = new RenderAC3D(filename);
        getCache().add(filename, render, moddate);
        return new Drawable(getCache().get(filename));
    } else if (filetype == ".TRI") {
        Render *render = new RenderTriSoup(filename);
        getCache().add(filename, render, moddate);
        return new Drawable(getCache().get(filename));
    } else if (filetype == ".OBJ") {
        Render *render = new RenderOBJ(filename);
        getCache().add(filename, render, moddate);
        return new Drawable(getCache().get(filename));
    } else if (filetype == ".IVG") {
        Render *render = new RenderIVG(filename);
        getCache().add(filename, render, moddate);
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
